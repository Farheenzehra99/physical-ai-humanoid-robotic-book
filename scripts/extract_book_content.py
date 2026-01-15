from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from bs4 import BeautifulSoup
import re
import os
import time
from urllib.parse import urljoin, urlparse
import json

def get_webdriver():
    """Initialize and return a Chrome WebDriver with appropriate options"""
    chrome_options = Options()
    chrome_options.add_argument("--headless")  # Run in background
    chrome_options.add_argument("--no-sandbox")
    chrome_options.add_argument("--disable-dev-shm-usage")
    chrome_options.add_argument("--disable-gpu")
    chrome_options.add_argument("--window-size=1920,1080")

    # You might need to install chromedriver separately or use webdriver-manager
    try:
        from webdriver_manager.chrome import ChromeDriverManager
        from selenium.webdriver.chrome.service import Service
        service = Service(ChromeDriverManager().install())
        driver = webdriver.Chrome(service=service, options=chrome_options)
    except:
        # If webdriver-manager fails, try without it (assuming chromedriver is in PATH)
        driver = webdriver.Chrome(options=chrome_options)

    return driver

def extract_meaningful_content(url):
    """
    Extract meaningful content from a URL, removing scripts, styles,
    navigation bars, headers, footers, and ads. Uses Selenium to handle JS-rendered content.
    """
    driver = None
    try:
        driver = get_webdriver()
        driver.get(url)

        # Wait for the page to load
        WebDriverWait(driver, 10).until(
            EC.presence_of_element_located((By.TAG_NAME, "body"))
        )

        # Additional wait for dynamic content to load (adjust as needed)
        time.sleep(3)

        # Get the page source after JavaScript has rendered the content
        html = driver.page_source
        soup = BeautifulSoup(html, 'html.parser')

        # Remove unwanted elements
        for element in soup(['script', 'style', 'nav', 'header', 'footer',
                           'aside', 'ad', 'advertisement', 'iframe',
                           'noscript', 'img', 'svg', 'button']):
            element.decompose()

        # Remove elements with common class names for navigation/advertisements
        for element in soup.find_all(class_=re.compile(r'nav|menu|header|footer|sidebar|ad|banner|social|share|comment')):
            element.decompose()

        # Look for main content areas - trying more specific selectors for Docusaurus sites
        main_content = (
            soup.find('main') or
            soup.find('article') or
            soup.find('div', class_=re.compile(r'container|theme|doc|content|main|article|post|body|markdown')) or
            soup.find('div', {'id': re.compile(r'main|content|doc|article')}) or
            soup.find('div', attrs={'role': 'main'}) or
            soup.find('div', class_=re.compile(r'docs-content|doc-content|theme')) or
            soup.find('body')
        )

        if main_content:
            # Extract text content
            text = main_content.get_text(separator='\n', strip=True)
            # Clean up excessive whitespace
            text = re.sub(r'\n\s*\n', '\n\n', text)
            return text.strip()
        else:
            # Fallback to body content if no main content area found
            body = soup.find('body')
            if body:
                text = body.get_text(separator='\n', strip=True)
                text = re.sub(r'\n\s*\n', '\n\n', text)
                return text.strip()
            else:
                # If no body, use the entire soup
                text = soup.get_text(separator='\n', strip=True)
                text = re.sub(r'\n\s*\n', '\n\n', text)
                return text.strip()

    except Exception as e:
        print(f"Error extracting content from {url}: {str(e)}")
        return ""
    finally:
        if driver:
            driver.quit()

def categorize_url(url):
    """
    Categorize the URL into module and chapter based on the path structure
    """
    parsed = urlparse(url)
    path_parts = parsed.path.strip('/').split('/')

    # Default values
    module = "general"
    chapter = "general"
    page = "page"

    # Extract module and chapter from path
    for i, part in enumerate(path_parts):
        if part.startswith('module'):
            if i + 1 < len(path_parts):
                module = part
                if path_parts[i+1].startswith('chapter'):
                    chapter = path_parts[i+1]
        elif 'chapter' in part:
            chapter = part

    # Extract page name
    if path_parts and path_parts[-1]:
        page = path_parts[-1]

    return module, chapter, page

def process_sitemap_urls(urls):
    """
    Process all URLs from the sitemap, extract content, and organize by module/chapter
    """
    content_by_module = {}

    for i, url in enumerate(urls):
        print(f"Processing {i+1}/{len(urls)}: {url}")

        # Extract content
        content = extract_meaningful_content(url)

        # Skip if content is less than 200 characters
        if len(content) < 100:
            print(f"  Skipping {url} - insufficient content ({len(content)} chars)")
            continue

        # Categorize the URL
        module, chapter, page = categorize_url(url)

        # Initialize module and chapter in the dictionary
        if module not in content_by_module:
            content_by_module[module] = {}

        if chapter not in content_by_module[module]:
            content_by_module[module][chapter] = []

        # Add content with metadata
        content_by_module[module][chapter].append({
            'url': url,
            'page': page,
            'content': content
        })

        # Be respectful to the server
        time.sleep(0.5)

    return content_by_module

def save_content_organized(content_by_module):
    """
    Save content organized by modules and chapters in the specified structure
    """
    base_dir = "book_data"

    for module, chapters in content_by_module.items():
        module_dir = os.path.join(base_dir, module)
        os.makedirs(module_dir, exist_ok=True)

        for chapter, pages in chapters.items():
            # Create a single file for each chapter combining all pages
            chapter_file = os.path.join(module_dir, f"{chapter}.txt")

            with open(chapter_file, 'w', encoding='utf-8') as f:
                for page_data in pages:
                    f.write(f"URL: {page_data['url']}\n")
                    f.write(f"Page: {page_data['page']}\n")
                    f.write("-" * 80 + "\n")
                    f.write(page_data['content'])
                    f.write("\n\n" + "="*80 + "\n\n")

            print(f"Saved: {chapter_file} ({len(pages)} pages)")

def main():
    # List of URLs from sitemap
    urls = [
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/auth/dashboard',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/auth/profile',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/auth/signin',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/auth/signup',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/auth/verify-email',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/tags',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/tags/installation',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/tags/isaac-sim',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/tags/nvidia',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/tags/omniverse',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/tags/physical-ai',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/tags/robotics-simulation',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/tags/setup',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/appendix/cloud-gpu-setup',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/appendix/contributing',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/appendix/hardware-alternatives',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/appendix/troubleshooting',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/category/chapter-10-capstone---autonomous-humanoid',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/category/chapter-8-voice-to-action',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/category/chapter-9-cognitive-planning',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/introduction/INTRODUCTION',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-02-ros2-architecture-overview',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-03-nodes-topics-services-explained',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-03b-nodes-topics-services-practical',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-02-python-agents-ros/page-04-python-agents-overview',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-02-python-agents-ros/page-04b-python-agents-integration',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-02-python-agents-ros/page-05-introduction-to-rclpy',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-02-python-agents-ros/page-06-python-agent-controls-humanoid-arm',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-03-urdf-humanoids/page-07-what-is-urdf',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-03-urdf-humanoids/page-08-urdf-components',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-01-foundations/chapter-03-urdf-humanoids/page-09-writing-your-first-urdf',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-02-digital-twin/chapter-04-gazebo-physics-simulation/page-10-why-simulation-matters',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-02-digital-twin/chapter-04-gazebo-physics-simulation/page-11-gazebo-basics',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-02-digital-twin/chapter-04-gazebo-physics-simulation/page-12-urdf-sdf-robot-description',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-02-digital-twin/chapter-05-unity-rendering/page-13-why-unity-for-robotics',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-02-digital-twin/chapter-05-unity-rendering/page-14-unity-scene-setup-basics',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-02-digital-twin/chapter-05-unity-rendering/page-15-simulating-sensors-lidar-cameras-imus',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/assets/',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/page-20-setting-up-isaac-sdk',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-08-voice-to-action/page-01-intro-speech-robotics',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-08-voice-to-action/page-02-whisper-architecture',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-08-voice-to-action/page-03-whisper-setup',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-08-voice-to-action/page-04-voice-command-pipeline',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-08-voice-to-action/page-05-ros2-integration',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-08-voice-to-action/page-06-exercises',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-01-intro-cognitive-robotics',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-02-llm-task-planning',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-03-natural-language-actions',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-04-ros2-action-generation',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-05-exercises',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-01-project-overview',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-02-system-architecture',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-03-implementation-guide',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-04-testing-validation',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-05-demo-presentation',
        'https://physical-ai-humanoid-robotic-book-ten.vercel.app/'
    ]

    # Filter out authentication and tag pages, keep only the book content
    book_urls = [url for url in urls if '/docs/' in url and
                 not any(skip in url for skip in ['/auth/', '/tags/', '/category/'])]

    print(f"Processing {len(book_urls)} book content URLs...")

    # Process the URLs
    content_by_module = process_sitemap_urls(book_urls)

    # Save the organized content
    save_content_organized(content_by_module)

    print(f"\nProcessing complete! Content saved in the 'book_data' directory.")

    # Print summary
    total_pages = 0
    for module, chapters in content_by_module.items():
        print(f"\nModule: {module}")
        for chapter, pages in chapters.items():
            print(f"  Chapter: {chapter} ({len(pages)} pages)")
            total_pages += len(pages)

    print(f"\nTotal processed pages: {total_pages}")

if __name__ == "__main__":
    main()