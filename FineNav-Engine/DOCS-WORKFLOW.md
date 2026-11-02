# FineNav 开发者文档工作流

## 1. 范围与组织级职责

面向开发者的文档使用 Markdown 和 MkDocs，部署到 GitHub Pages。开发者指实际
使用、集成或扩展代码的人。面向非技术用户的文档使用对外开放的飞书知识库；
本工作流及后续文档讨论只针对开发者文档。

每个代码仓库的 `docs/` 是自己的文档源。公开的 `FineNav/FineNav.github.io`
是唯一的聚合与部署仓库，在构建时读取各仓库文档，合并成中英双语站点，
发布到 <https://finenav.github.io/>。
各模块只维护 Markdown、导航配置并 push，不单独部署网站。

```text
FineNav-Engine/docs/ ──┐
finenav_map/docs/ ─────┼── 聚合 CI ── MkDocs 双语构建 ── GitHub Pages
finenav_planner/docs/ ─┘
```

源码仓库可以保持私有；聚合仓库公开。选择进入聚合站点的文档对外公开，
私有仓库读取凭据只保存在聚合仓库的 Actions Secrets。

## 2. 文档目录与语言约定

```text
<repository>/
├── <source directories>/
├── docs/
│   ├── index.md
│   ├── index.zh.md
│   ├── architecture/
│   │   ├── overview.md
│   │   └── overview.zh.md
│   ├── guide/
│   │   ├── quickstart.md
│   │   └── quickstart.zh.md
│   ├── examples/
│   │   └── basic-usage.md
│   └── assets/
└── mkdocs.yml
```

- 英文默认，直接用 `.md`；中文用同目录、同名 `.zh.md`，不采用 `docs/en/` 和 `docs/zh/`。
- 中文缺页时回退到英文，不创建空的中文占位页。插件不会自动翻译正文。
- `architecture/` 放设计与架构，`guide/` 放安装、配置和使用，`examples/` 放案例。
- 图片放在文档目录内，尽量使用相对链接。导航引用英文文件，不单独重复列出中文页。
- 中英文的命令、包名、参数和 API 名称保持一致。
- Linux 终端命令每条使用单行，不使用反斜杠续行，确保可直接复制执行。

组织级目录和语言方案已经确定，不重复询问。只有首次设计具体页面导航或重构导航时，
才询问使用默认结构、现有结构还是指定结构；局部更新直接沿用现有导航。

默认内容骨架为 Overview、Quickstart、使用指南和示例。插件目录、自定义组件、
API 参考和排错页面按实际需要加入，不创建空分类。
FineNav-Engine 当前导航采用：**如何安装 → Getting Started 五步教程 → finenav_planner → finenav_map**。
五步教程依次为：运行示例、选择 Map/Planner、编写 WorldView、组装控制 Pipeline、按需求配置行为树。
教程以当前 `finenav_example` 源码为依据，命令和接口随示例更新；自定义 Planner/Map 指南放在对应组件栏目下。
底层实现原理独立放架构文档。聚合站另为 FineNav-SlamBox、FineNav（ICRA 版本）保留占位页。
插件具体说明由对应模块仓库维护，生态首页与 Engine 指南提供目录和通用集成入口。

## 3. 各模块的 mkdocs.yml

根目录配置声明文档目录和导航，也可用于模块本地预览。聚合器读取导航等导入信息，
不自动继承模块的主题、插件或导航翻译；最终站点配置由聚合仓库统一维护。

以下示例使用 static-i18n 1.x 配置：

```yaml
site_name: FineNav Core Framework
docs_dir: docs
theme:
  name: material
nav:
  - Overview: index.md
  - Architecture:
      - Overview: architecture/overview.md
  - Guide:
      - Quickstart: guide/quickstart.md
  - Examples:
      - Basic Usage: examples/basic-usage.md
plugins:
  - search
  - i18n:
      docs_structure: suffix
      fallback_to_default: true
      languages:
        - locale: en
          name: English
          default: true
        - locale: zh
          name: 中文
          nav_translations:
            Overview: 总览
            Architecture: 架构设计
            Guide: 使用指南
            Quickstart: 快速开始
            Examples: 实际案例
            Basic Usage: 基础用法
```

按真实文件裁剪导航。1.x 使用语言条目内的 `default: true`，不要混用旧版顶层
`default_language`；每种语言必须填写 `name`。
各模块不需要 Pages、部署工作流或部署密钥。OpenWiki 工作流可选，与站点部署分开。

## 4. 聚合仓库配置

```text
FineNav.github.io/
├── docs/
│   ├── index.md
│   ├── index.zh.md
│   └── assets/
├── mkdocs.yml
├── requirements-docs.txt
└── .github/workflows/deploy-docs.yml
```

聚合仓库自己的 `docs/` 维护生态首页和公共资源，不手工复制维护模块正文。
multirepo 在构建时读取模块文档和导航。

以下使用用户示例里的占位仓库名。实施时替换为真实仓库并核对分支；
Engine 当前文档基于 `main`；聚合配置与安装命令均使用 `main`，其他仓库的分支需分别核对。

```yaml
site_name: FineNav Documentation
site_url: https://finenav.github.io/
docs_dir: docs
site_dir: site
use_directory_urls: true
theme:
  name: material
  palette:
    primary: teal
    accent: red
  features:
    - navigation.sections
    - navigation.indexes
    - content.code.copy
plugins:
  - search
  - multirepo:
      cleanup: true
      keep_docs_dir: false
  - i18n:
      docs_structure: suffix
      fallback_to_default: true
      languages:
        - locale: en
          name: English
          default: true
        - locale: zh
          name: 中文
          site_name: FineNav 文档
          nav_translations:
            Ecosystem Overview: 生态总览
            core-framework: 核心框架
            module-a: 模块 A
            module-b: 模块 B
            Overview: 总览
            Architecture: 架构设计
            Guide: 使用指南
            Quickstart: 快速开始
            Examples: 实际案例
            Basic Usage: 基础用法
nav:
  - Ecosystem Overview: index.md
  - core-framework: '!import https://github.com/FineNav/core-framework?branch=main'
  - module-a: '!import https://github.com/FineNav/module-a?branch=main'
  - module-b: '!import https://github.com/FineNav/module-b?branch=main'
```

关键约定：

- `!import` 必须作为加引号的字符串，不是 YAML 自定义标签。它指向仓库，默认导入
  `docs/*`，读取模块的 `mkdocs.yml` 导航。
- 明确指定 `?branch=...`，不依赖插件默认的 `master`。
- `keep_docs_dir: false` 与不带 `/docs/` 的预期 URL 配套。true 会保留 `docs/` 层级，
  导入导航也需要调整对应路径。
- multirepo 在 i18n 前注册，让导入文档参与语言处理；搜索显式启用。
- 聚合配置统一声明所需导航翻译、代码高亮、Mermaid 等扩展及依赖。

实际组合构建发现：multirepo 0.8.3 提供的文件位于文档源目录外，static-i18n 1.3.1
会忽略这些 Markdown。聚合仓库已加入 `scripts/multirepo_i18n.py` 兼容钩子，
在语言处理前自动将导入文件临时放入文档目录，完成全部语言构建后清理。
聚合配置还需要 `hooks: [scripts/multirepo_i18n.py]`；模块无需增加任何步骤。
完整执行配置以 [聚合仓库](https://github.com/FineNav/FineNav.github.io) 为准。

按查询版本的兼容声明，候选依赖如下。static-i18n 1.3.1 的 Material 可选依赖
限制为 `<9.7.2`；正式采用前需验证组合构建，成功后固定实际使用版本。

```text
mkdocs>=1.6,<2
mkdocs-material>=9.6,<9.7.2
mkdocs-multirepo-plugin==0.8.3
mkdocs-static-i18n==1.3.1
```

## 5. URL 规则

普通 MkDocs 页面 URL 来自文档文件路径，不是普通导航标题。
但 multirepo 0.8.3 的 `!import` 会将所在章节路径经 `slugify` 转为导入目录前缀。
因此采用稳定英文导入标识，通过 `nav_translations` 显示中文。
直接使用“核心框架”作为导入标识可能生成音译前缀，不能保证得到 `/core-framework/`
或原样的中文路径。改变导入标识会改变 URL。

上面的配置预期生成：

| 内容 | 英文 | 中文 |
| --- | --- | --- |
| 生态首页 | `/` | `/zh/` |
| 核心框架首页 | `/core-framework/` | `/zh/core-framework/` |
| 架构概述 | `/core-framework/architecture/overview/` | `/zh/core-framework/architecture/overview/` |
| 模块 A 快速开始 | `/module-a/guide/quickstart/` | `/zh/module-a/guide/quickstart/` |

英文默认在根路径，中文在 `/zh/`。中文缺页时回退英文内容。
i18n 提供 Material 语言切换集成；最终链接、图片、语言切换、搜索和回退行为
必须通过真实聚合构建检查，不能仅凭示例认定已验证。

## 6. 私有模块读取权限

聚合仓库的默认 `GITHUB_TOKEN` 通常不能读取其他私有仓库。
在聚合仓库设置 `DOCS_READ_TOKEN` Secret，使用组织允许的细粒度 PAT 或 GitHub App
安装令牌，对导入的模块仓库授予 Contents 只读权限，并完成组织要求的审核或 SSO 授权。

multirepo 0.8.3 使用大小写敏感的 `GithubAccessToken` 环境变量。
在构建步骤把 `secrets.DOCS_READ_TOKEN` 映射到它，不把令牌写入文档、配置、URL 或日志。
GitHub App 方案需另加生成短期安装令牌的步骤。
Pages 部署使用聚合仓库自己的 `GITHUB_TOKEN` 和 OIDC 权限。
模块不需要部署密钥，也无需单独启用 Pages。

## 7. 触发与唯一部署入口

聚合仓库 **Settings → Pages → Source** 设置为 **GitHub Actions**。
只采用定时和手动触发：每日 UTC 06:00，即北京时间 14:00，或 Actions 页面 Run workflow。
GitHub 定时任务可能延迟；工作流必须位于聚合仓库默认分支，定时运行也以默认分支为准。
各模块 push 后等待下一次聚合构建，不立即发布，也不需要配置跨仓库触发工作流。

以下为实施模板，需先准备已核对的聚合配置、依赖和 Secret：

```yaml
name: Deploy FineNav documentation
on:
  schedule:
    - cron: '0 6 * * *'
  workflow_dispatch:
permissions:
  contents: read
  pages: write
  id-token: write
concurrency:
  group: finenav-pages
  cancel-in-progress: false
jobs:
  deploy:
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - name: Checkout aggregator
        uses: actions/checkout@v4
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - name: Install dependencies
        run: python -m pip install -r requirements-docs.txt
      - name: Configure Pages
        uses: actions/configure-pages@v5
      - name: Build bilingual site
        env:
          GithubAccessToken: ${{ secrets.DOCS_READ_TOKEN }}
        run: python -m mkdocs build --strict
      - name: Upload Pages artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: site
      - name: Deploy Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

CI 只上传生成的 `site/`，不创建网站分支。构建失败就不执行部署，保留上次成功站点。
建议记录本次读取的模块提交；可重复的正式发布使用已确认的发布标签。

## 8. 正文生产与验证

面向开发者的使用正文直接说明组件功能和使用方法。源码链接统一指向各仓库的 `main`，
省略默认分支、核对提交号和其他分支说明；仅在影响安装或兼容性时说明版本差异。

每次记录仓库、分支/提交、开发者任务和已验证环境。源码、README、包清单、构建配置
和真实示例是技术依据。OpenWiki 只在有助于解决具体问题时按需检索，结论再对照源码；
不需要每次修改初始化或刷新整个 wiki。

操作指南按“目标 → 前置条件 → 步骤 → 预期结果 → 必要排错”组织。
区分代码片段与完整示例，不编造插件功能、兼容关系或安装成功结果。
同步更新中英文；暂时缺中文时允许英文回退，交付说明列明缺失翻译。

验证分层进行：

1. 模块本地预览：检查 Markdown、导航、中文页面与英文回退。
2. 真实聚合构建：指定实际分支执行 `mkdocs build --strict`，检查 URL、跨模块链接、
   资源、语言切换和搜索。
3. 项目运行验证：在支持环境中从全新工作空间下载、安装依赖、编译并运行示例。
4. 发布后访问：检查公开首页、模块路径、`/zh/` 和资源加载。

网页构建通过不代表 C++ 编译或导航成功。FineNav 当前 Quickstart 的 Ubuntu 22.04 /
ROS 2 Humble 命令尚未完成端到端试跑，Engine CI 不证明 simulator 运行成功。
DocuMCP 1.0.0 的现有实验正文填充失败，C++ 上下文生成不受支持，不是必备环节。

## 9. 从当前版本迁移

Engine 的已有使用指南现已整理到 `docs/` 和 `docs/guide/`，十个现有页面均有同名
`.zh.md` 中文版。`mkdocs.yml` 声明导航并启用语言后缀与缺页英文回退。
旧详细版完整保存在 `docs-detailed/`，详细版配置和 OpenWiki 导出脚本已指向该目录。
`docs-user/` 保留为非活动的历史副本，后续修改以 `docs/` 为准。
Engine 的远程 `gh-pages` 已删除；公开聚合仓库为 `FineNav/FineNav.github.io`，
部署状态以聚合仓库的 Actions 运行结果为准。
Engine 本地英文站、中文站和保留的旧详细版均已通过严格构建；
双语页面与缺页英文回退机制已检查。聚合仓库从 Engine 的远程 `main` 分支
导入并严格构建双语站点已通过，内部页面和资源链接已检查；线上状态查看其 Actions。

实施顺序：

1. 已保存本次迁移前的用户版、详细版和配置，已有备份不覆盖。
2. 已将详细版保留到 `docs-detailed/`，同步调整配置、导出路径和维护说明。
3. 已迁入英文指南并更新导航与相对链接，现有十页的 `.zh.md` 翻译已补齐。
4. 核对 map/planner 实际文档分支和模块导航，由各仓库维护自己的源文件。
5. 创建或复用公开的 `FineNav/FineNav.github.io`，添加生态首页、聚合配置和 CI。
6. 配置最小只读凭据与 Pages，先手动运行验证，再依赖定时更新。

已有保留版本：

- `F:\FineNav-Engine-docs-versions\detailed-v1`。
- `F:\FineNav-Engine-docs-versions\user-v1-2026-10-12`。
- `F:\FineNav-Engine-docs-versions\workflow-before-aggregation-2026-10-13.md`。
- `F:\FineNav-Engine-docs-versions\user-before-aggregation-2026-10-13`。

配置依据为 2026-10-13 查询的 multirepo 0.8.3、static-i18n 1.3.1 发行说明和源码。
源码核对不等于已完成实际 FineNav 跨仓库双语构建或线上部署。

## 可复用任务描述

```text
按 DOCS-WORKFLOW.md 更新 FineNav 开发者文档。
涉及仓库与分支/提交：<输入>。
本次任务与读者操作：<输入>。
各仓库以 docs/ 为源，英文 .md、中文同目录 .zh.md。
沿用已确认导航；首次设计或重构具体导航才询问默认/现有/指定结构。
先核对源码和真实示例，OpenWiki 为可选资料来源。
架构放 architecture/，使用指南放 guide/，案例放 examples/。
由 FineNav.github.io 聚合 CI 定时或手动构建，各模块不单独部署。
大幅迁移前保存旧版，完成适用的验证，报告执行结果和未验证项。
```

## 参考资料

- [multirepo 0.8.3 发行说明](https://pypi.org/project/mkdocs-multirepo-plugin/0.8.3/)
- [static-i18n 快速开始](https://ultrabug.github.io/mkdocs-static-i18n/getting-started/quick-start/)
- [static-i18n 1.x 升级说明](https://ultrabug.github.io/mkdocs-static-i18n/setup/upgrading-to-1/)
