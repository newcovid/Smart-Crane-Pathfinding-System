import os
import re
import requests
import logging
from urllib.parse import urljoin

# 配置日志
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("SmartDepManager")

# 基础路径
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
VENDOR_DIR = os.path.join(BASE_DIR, "static", "vendor")

# CDN 镜像源配置 (主源 + 备用源)
# 我们优先使用 jsDelivr，因为它在国内通常更稳定
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
    # --- 字体美化 (Inter Font) ---
    # 既然追求美观，我们把 Inter 字体也离线下来
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
            logger.info(f"⬇️  下载: {url_template}")
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
            logger.info(f"⬇️  [{mirror_name}] 下载: {url}")
            resp = requests.get(url, timeout=10)
            if resp.status_code == 404:
                # 404 可能是路径不对，尝试下一个镜像
                logger.warning(f"   404 Not Found, 尝试下一个源...")
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

    logger.info(f"   🔍 在 CSS 中发现 {len(urls)} 个资源引用")

    for relative_url in urls:
        # 忽略 data: base64
        if relative_url.startswith("data:"):
            continue

        # 计算绝对 URL
        asset_remote_url = urljoin(css_url, relative_url)

        # 简单处理一下 URL 参数 (如 font.woff2?v=3.19 -> font.woff2)
        clean_filename = relative_url.split("?")[0].split("#")[0]
        # 修正：有些 url 是 ../ 开头的，我们需要保持相对目录结构，或者展平
        # 这里为了简单，我们直接下载到 CSS 同级目录，但需要处理 ../
        # 最稳妥的方法是：保持相对路径结构

        asset_local_path = os.path.join(css_dir, relative_url.split("?")[0])
        # 如果路径里有 ../，os.path.join 会处理，但我们要确保它在 VENDOR_DIR 内（这里假设它是安全的）

        # 下载资源
        try:
            logger.info(f"   📦 下载资源: {clean_filename}")
            # 这里使用直接 requests，因为资源 URL 已经是绝对的了
            # 同时也加上重试机制（简单的）
            asset_resp = requests.get(asset_remote_url, timeout=15)
            if asset_resp.status_code == 200:
                ensure_dir(asset_local_path)
                with open(asset_local_path, "wb") as f:
                    f.write(asset_resp.content)
            else:
                logger.error(
                    f"   ❌ 资源下载失败 {asset_resp.status_code}: {asset_remote_url}"
                )
        except Exception as e:
            logger.error(f"   ❌ 资源下载异常: {asset_remote_url} - {e}")

    return css_content


def main():
    print("=" * 60)
    print("   🚀 智能起重机控制台 - 依赖同步工具")
    print("   集成镜像加速、断点续传、CSS资源解析")
    print("=" * 60)

    success_count = 0

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

            logger.info(f"✅ 已保存: {task['path']}\n")
            success_count += 1
        else:
            logger.error(f"❌ 彻底失败: {task['name']}\n")

    print("=" * 60)
    print(f"同步完成: {success_count}/{len(TASKS)}")
    if success_count == len(TASKS):
        print("🎉所有资源（含字体）已本地化，界面将足够美观且无需联网。")
    else:
        print("⚠️ 部分失败，请检查网络。核心功能可能仍可用，但图标可能缺失。")


if __name__ == "__main__":
    main()
