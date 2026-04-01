// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <oneapi/tbb/partitioner.h>

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "finenav_util/algo_configurator.hpp"
#include "finenav_util/node_thread.hpp"

namespace finenav {

template <typename MapT>
class LockedMapRO {
  public:
    // 构造时自动加读锁 (Shared Lock)
    LockedMapRO(const MapT& map, std::shared_mutex& mutex) : map_(map), lock_(mutex) {}

    // 允许通过 -> 操作符直接调用 MapT 的 const 方法 (极度丝滑)
    const MapT* operator->() const { return &map_; }

    const MapT& get() const { return map_; }

  private:
    const MapT& map_;
    // 离开作用域时，自动释放读锁
    std::shared_lock<std::shared_mutex> lock_;
};

template <typename MapT>
class LockedMapRW {
  public:
    // 构造时自动加写锁 (Unique Lock)
    LockedMapRW(MapT& map, std::shared_mutex& mutex) : map_(map), lock_(mutex) {}

    // 允许通过 -> 操作符直接调用 MapT 的方法
    MapT* operator->() { return &map_; }

    MapT& get() { return map_; }

  private:
    MapT& map_;
    // 离开作用域时，自动释放写锁
    std::unique_lock<std::shared_mutex> lock_;
};

template <typename MapT>
class MapServer {
  public:
    struct SourceBufferPolicy {
        std::size_t max_buffer_size{32};
        double observation_keep_time_sec{0.0};
        double source_timeout_sec{0.0};
    };

    using PreUpdateHook = std::function<void(MapT& map)>;
    using PostUpdateHook = std::function<void(const MapT& map)>;

    template <typename MsgT>
    using Fuser = std::function<void(const MsgT& msg, MapT& map)>;

    /**
     * @brief 构造 MapServer，初始化内部定时器以定期执行地图更新。
     * @param node ROS 2 节点的指针
     * @param server_name MapServer 的名称
     * @param update_rate_hz 地图更新周期的频率 (Hz)
     */
    explicit MapServer(rclcpp::Node::SharedPtr node, const std::string& server_name, double update_rate_hz = 10.0);

    /**
     * @brief 注册在每次地图更新周期前执行的回调函数。
     * @param hook 用户定义的回调函数
     */
    void registerPreUpdateHook(std::function<void(MapT& map)> hook);

    /**
     * @brief 注册在每次地图更新周期后执行的回调函数。
     * @param hook 用户定义的回调函数
     */
    void registerPostUpdateHook(std::function<void(MapT& map)> hook);

    /**
     * @brief 获取带有并发保护的 RAII 读视图，保证访问地图时的绝对线程安全。
     * @return 返回受读锁保护的地图视图对象
     */
    LockedMapRO<MapT> getLockedReadView();

    /**
     * @brief 获取带有并发保护的 RAII 读写视图，保证访问地图时的绝对线程安全。用于主动且即时地修改地图。
     * @return 返回受写锁保护的地图视图对象
     */
    LockedMapRW<MapT> getLockedWriteView();

    /**
     * @brief 注册更新源。返回一个线程安全的回复函数，用于主动传入数据。强制使用右值引用接收数据以便转移所有权。
     * @tparam MsgT 更新源消息的数据类型
     * @param source_name 数据源的名称
     * @param policy 规定该源的缓存与超时策略
     * @param fuser 用户提供的数据融合函数，将 MsgT 数据融入 MapT 中
     * @return std::function<void(MsgT&&, const rclcpp::Time&)> 线程安全的数据推送闭包
     */
    template <typename MsgT>
    std::function<void(MsgT&&, const rclcpp::Time&)> addUpdateSource(
        const std::string& source_name, SourceBufferPolicy policy, Fuser<MsgT> fuser);

  private:
    void runUpdateCycle(const rclcpp::Time& now);

    class IUpdateSource {
      public:
        virtual ~IUpdateSource() = default;
        virtual void applyBuffered(MapT& map, const rclcpp::Time& now) = 0;
    };

    template <typename MsgT>
    class UpdateSource final : public IUpdateSource {
      public:
        UpdateSource(const std::string& source_name, SourceBufferPolicy policy, Fuser<MsgT> fuser);

        void enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp);
        void applyBuffered(MapT& map, const rclcpp::Time& now) override;

      private:
        struct BufferedMessage {
            rclcpp::Time stamp;
            MsgT msg;
        };

        void enqueue(MsgT&& msg, const rclcpp::Time& stamp);
        void pruneExpired(const rclcpp::Time& now);

        std::string source_name_;
        SourceBufferPolicy policy_;
        Fuser<MsgT> fuser_;
        std::mutex mutex_;
        std::deque<BufferedMessage> buffer_;
        rclcpp::Time last_seen_stamp_{0, 0, RCL_ROS_TIME};
        bool has_seen_message_{false};
    };

    template <typename T>
    void keep_alive(std::shared_ptr<T> handle);

    rclcpp::Node::SharedPtr node_;
    std::string server_name_;

    rclcpp::Node::SharedPtr pnode_;
    std::unique_ptr<NodeThread> pnode_thread_;

    // 核心数据与锁
    MapT map_instance_;
    std::shared_mutex rw_mutex_;  // C++17 读写锁

    // 回调列表与生命周期管理
    std::vector<std::function<void(MapT&)>> pre_update_hooks_;
    std::vector<std::function<void(MapT&)>> post_update_hooks_;
    std::vector<std::shared_ptr<void>> held_param_handles_;
    std::vector<std::shared_ptr<IUpdateSource>> update_sources_;
    rclcpp::TimerBase::SharedPtr update_timer_;
};

// ==============================================================================
// Implementation
// ==============================================================================

template <typename MapT>
MapServer<MapT>::MapServer(rclcpp::Node::SharedPtr node, const std::string& server_name, double update_rate_hz)
    : node_(node), server_name_(server_name) {
    pnode_ = rclcpp::Node::make_shared(server_name_, node_->get_name(), rclcpp::NodeOptions());
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    keep_alive(AlgoConfigurator<MapT>::load(pnode_, map_instance_));

    auto period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / update_rate_hz));

    update_timer_ = rclcpp::create_timer(pnode_, node_->get_clock(), period,
                                         [this]() { this->runUpdateCycle(this->node_->now()); });
}

template <typename MapT>
void MapServer<MapT>::registerPreUpdateHook(std::function<void(MapT& map)> hook) {
    pre_update_hooks_.push_back(std::move(hook));
}

template <typename MapT>
void MapServer<MapT>::registerPostUpdateHook(std::function<void(MapT& map)> hook) {
    post_update_hooks_.push_back(std::move(hook));
}

template <typename MapT>
LockedMapRO<MapT> MapServer<MapT>::getLockedReadView() {
    return LockedMapRO<MapT>(map_instance_, rw_mutex_);
}

template <typename MapT>
LockedMapRW<MapT> MapServer<MapT>::getLockedWriteView() {
    return LockedMapRW<MapT>(map_instance_, rw_mutex_);
}

template <typename MapT>
template <typename MsgT>
std::function<void(MsgT&&, const rclcpp::Time&)> MapServer<MapT>::addUpdateSource(
    const std::string& source_name, SourceBufferPolicy policy, Fuser<MsgT> fuser) {

    auto source = std::make_shared<UpdateSource<MsgT>>(source_name, policy, std::move(fuser));
    update_sources_.push_back(source);

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "MapServer added update source: " << source_name);

    std::weak_ptr<UpdateSource<MsgT>> weak_source = source;
    return [weak_source](MsgT&& msg, const rclcpp::Time& stamp) {
        if (auto src = weak_source.lock()) {
            src->enqueueThreadSafe(std::move(msg), stamp);
        }
    };
}

template <typename MapT>
void MapServer<MapT>::runUpdateCycle(const rclcpp::Time& now) {
    std::unique_lock<std::shared_mutex> lock(rw_mutex_);

    for (auto& hook : pre_update_hooks_) {
        hook(map_instance_);
    }

    for (auto& source : update_sources_) {
        source->applyBuffered(map_instance_, now);
    }

    for (auto& hook : post_update_hooks_) {
        hook(map_instance_);
    }
}

template <typename MapT>
template <typename T>
void MapServer<MapT>::keep_alive(std::shared_ptr<T> handle) {
    if (handle)
        held_param_handles_.push_back(handle);
}

// ==============================================================================
// 4. UpdateSource 成员定义
// ==============================================================================

template <typename MapT>
template <typename MsgT>
MapServer<MapT>::UpdateSource<MsgT>::UpdateSource(const std::string& source_name, SourceBufferPolicy policy,
                                                  Fuser<MsgT> fuser)
    : source_name_(source_name), policy_(policy), fuser_(std::move(fuser)) {}

template <typename MapT>
template <typename MsgT>
void MapServer<MapT>::UpdateSource<MsgT>::enqueueThreadSafe(MsgT&& msg,
                                                            const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    enqueue(std::move(msg), stamp);
}

template <typename MapT>
template <typename MsgT>
void MapServer<MapT>::UpdateSource<MsgT>::applyBuffered(MapT& map, const rclcpp::Time& now) {
    std::deque<BufferedMessage> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);

        pruneExpired(now);

        if (policy_.source_timeout_sec > 0.0 && has_seen_message_) {
            const auto elapsed = (now - last_seen_stamp_).seconds();
            if (elapsed > policy_.source_timeout_sec) {
                if (!buffer_.empty()) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("MapServer"), "MapServer UpdateSource '"
                                                                            << source_name_ << "' has timed out ("
                                                                            << elapsed << "s). Buffer cleared!");
                }
                buffer_.clear();
            }
        }
        pending.swap(buffer_);
    }

    if (!fuser_) {
        return;
    }

    if (!pending.empty()) {
        RCLCPP_DEBUG_STREAM(
            rclcpp::get_logger("MapServer"),  // TODO: 文件头定义 LOGGER
            "UpdateSource '" << source_name_ << "' applying " << pending.size() << " buffered messages.");
    }

    for (const auto& item : pending) {
        fuser_(item.msg, map);
    }
}

template <typename MapT>
template <typename MsgT>
void MapServer<MapT>::UpdateSource<MsgT>::enqueue(MsgT&& msg, const rclcpp::Time& stamp) {
    if (policy_.max_buffer_size > 0 && buffer_.size() >= policy_.max_buffer_size) {
        buffer_.pop_front();
    }
    buffer_.push_back(BufferedMessage{stamp, std::move(msg)});
    last_seen_stamp_ = stamp;
    has_seen_message_ = true;
}

template <typename MapT>
template <typename MsgT>
void MapServer<MapT>::UpdateSource<MsgT>::pruneExpired(const rclcpp::Time& now) {
    if (policy_.observation_keep_time_sec <= 0.0) {
        return;
    }

    while (!buffer_.empty()) {
        const auto age = (now - buffer_.front().stamp).seconds();
        if (age <= policy_.observation_keep_time_sec) {
            break;
        }
        buffer_.pop_front();
    }
}

}  // namespace finenav
