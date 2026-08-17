"""前端依赖离线本地化脚本。

把 `TASKS` 中声明的第三方资源从 CDN 拉取到 `static/vendor/`，
使运行时不再访问外部网络。CSS 类型的任务会递归解析其中的 `url(...)`
引用，一并下载字体与图片。

需要 `requirements-dev.txt` 中的 `requests`。全部任务成功时退出码为 0，
存在失败时为 1。
"""

import os
import re
import sys
import logging
from urllib.parse import urljoin

import requests

# 配置日志
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("SmartDepManager")

# 基础路径
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
VENDOR_DIR = os.path.join(BASE_DIR, "static", "vendor")

# CDN 镜像源配置（主源 + 备用源），按声明顺序依次尝试
MIRRORS = {
    "jsdelivr": "https://cdn.jsdelivr.net/npm",
    "unpkg": "https://unpkg.com",
}

# 定义下载任务
# type: "direct" (直接下载) | "css_bundle" (下载CSS并自动抓取其中的字体/图片)
TASKS = [
    # --- 核心库 ---
    {
        "name": "tailwindcss.js",
        "path": "tailwindcss.js",
        "url": "https://cdn.tailwindcss.com",
        "type": "direct",
    },
    {
        "name": "socket.io.min.js",
        "path": "socket.io.min.js",
        "url": "https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.5/socket.io.min.js",
        "type": "direct",
    },
    {
        "name": "vue.global.js",
        "path": "vue.global.js",
        "url": "{mirror}/vue@3/dist/vue.global.js",
        "type": "direct",
    },
    {
        "name": "three.module.js",
        "path": "three.module.js",
        "url": "{mirror}/three@0.160.0/build/three.module.js",
        "type": "direct",
    },
    {
        "name": "OrbitControls.js",
        "path": "three/examples/jsm/controls/OrbitControls.js",
        "url": "{mirror}/three@0.160.0/examples/jsm/controls/OrbitControls.js",
        "type": "direct",
    },
    # --- 图标库 (Phosphor Icons) ---
    # 自动解析 CSS 下载 woff2 文件
    {
        "name": "phosphor-regular",
        "path": "phosphor/regular.css",
        "url": "{mirror}/@phosphor-icons/web@2.1.1/src/regular/style.css",
        "type": "css_bundle",
    },
    {
        "name": "phosphor-bold",
        "path": "phosphor/bold.css",
        "url": "{mirror}/@phosphor-icons/web@2.1.1/src/bold/style.css",
        "type": "css_bundle",
    },
    {
        "name": "phosphor-fill",
        "path": "phosphor/fill.css",
        "url": "{mirror}/@phosphor-icons/web@2.1.1/src/fill/style.css",
        "type": "css_bundle",
    },
    # --- 界面字体 (Inter) ---
    # 字体文件同样本地化，否则首屏会回落到系统字体并产生布局抖动
    {
        "name": "inter-font",
        "path": "fonts/inter.css",
        "url": "https://rsms.me/inter/inter.css",
        "type": "css_bundle",
    },
]


def ensure_dir(file_path):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)


def get_with_retry(url_template):
    """尝试使用不同的镜像源下载"""
    # 如果不是模板URL（没有{mirror}），直接请求
    if "{mirror}" not in url_template:
        try:
            logger.info(f"下载: {url_template}")
            resp = requests.get(url_template, timeout=15)
            resp.raise_for_status()
            return resp.content, url_template
        except Exception as e:
            logger.error(f"   下载失败: {e}")
            return None, None

    # 尝试每个镜像源
    for mirror_name, mirror_base in MIRRORS.items():
        url = url_template.format(mirror=mirror_base)
        try:
            logger.info(f"[{mirror_name}] 下载: {url}")
            resp = requests.get(url, timeout=10)
            if resp.status_code == 404:
                # 404 可能是路径不对，尝试下一个镜像
                logger.warning("   404 Not Found，尝试下一个源")
                continue
            resp.raise_for_status()
            return resp.content, url
        except Exception as e:
            logger.warning(f"   源 {mirror_name} 连接失败: {e}")
            continue

    return None, None


def process_css_bundle(css_content, css_url, local_css_rel_path):
    """
    解析 CSS，下载其中引用的 url(...) 资源
    """
    css_text = css_content.decode("utf-8")
    # 正则匹配 url('...') 或 url("...")
    urls = set(re.findall(r'url\([\'"]?([^\'"\)]+)[\'"]?\)', css_text))

    css_dir = os.path.dirname(os.path.join(VENDOR_DIR, local_css_rel_path))

    logger.info(f"  CSS 中发现 {len(urls)} 个资源引用")

    for relative_url in urls:
        # 忽略 data: base64
        if relative_url.startswith("data:"):
            continue

        # 计算绝对 URL
        asset_remote_url = urljoin(css_url, relative_url)

        # 简单处理一下 URL 参数 (如 font.woff2?v=3.19 -> font.woff2)
        clean_filename = relative_url.split("?")[0].split("#")[0]
        # 保持上游的相对目录结构，而非展平到同一层——
        # 展平会让两个同名字体文件互相覆盖。
        asset_local_path = os.path.join(css_dir, relative_url.split("?")[0])

        # 路径穿越校验：远程 CSS 里的 url(../../x) 会被 os.path.join 如实解析，
        # 可以写到 VENDOR_DIR 之外的任意位置。这些 CSS 来自第三方 CDN，
        # 不能假定其内容可信。
        resolved = os.path.realpath(asset_local_path)
        if os.path.commonpath([resolved, os.path.realpath(VENDOR_DIR)]) != os.path.realpath(
            VENDOR_DIR
        ):
            logger.error(f"  跳过越界资源路径: {relative_url} -> {resolved}")
            continue

        # 下载资源
        try:
            logger.info(f"  下载资源: {clean_filename}")
            # CSS 内的资源 URL 经 urljoin 后已是绝对地址，不适用镜像替换，
            # 因此直接请求，不走 get_with_retry 的多源逻辑。
            asset_resp = requests.get(asset_remote_url, timeout=15)
            if asset_resp.status_code == 200:
                ensure_dir(asset_local_path)
                with open(asset_local_path, "wb") as f:
                    f.write(asset_resp.content)
            else:
                logger.error(
                    f"  资源下载失败 {asset_resp.status_code}: {asset_remote_url}"
                )
        except Exception as e:
            logger.error(f"  资源下载异常: {asset_remote_url} - {e}")

    return css_content


def main():
    logger.info("开始同步前端依赖，共 %d 项，目标目录 %s", len(TASKS), VENDOR_DIR)

    success_count = 0
    failed = []

    for task in TASKS:
        local_path = os.path.join(VENDOR_DIR, task["path"])

        # 1. 下载主文件
        content, final_url = get_with_retry(task["url"])

        if content:
            ensure_dir(local_path)

            # 2. 如果是 CSS Bundle，额外解析并下载资源
            if task["type"] == "css_bundle":
                content = process_css_bundle(content, final_url, task["path"])

            # 3. 保存文件
            with open(local_path, "wb") as f:
                f.write(content)

            logger.info("已保存: %s", task["path"])
            success_count += 1
        else:
            logger.error("下载失败（已穷尽全部镜像）: %s", task["name"])
            failed.append(task["name"])

    logger.info("同步完成: %d/%d", success_count, len(TASKS))
    if failed:
        logger.error("以下资源缺失，界面图标或字体将无法离线加载: %s", ", ".join(failed))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
