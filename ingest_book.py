import os
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv
import time

load_dotenv()

# === Keys aur URLs ===
COHERE_API_KEY = os.getenv("COHERE_API_KEY")  
QDRANT_URL = os.getenv("QDRANT_URL")          
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "rag_full_book"

# SITEMAP_URL = "https://physical-ai-humanoid-robotic-book-kmg2eqtlm.vercel.app/sitemap.xml"

# Local file fallback
with open("sitemap.xml", "r", encoding="utf-8") as f:
    sitemap_content = f.read()

from bs4 import BeautifulSoup
soup = BeautifulSoup(sitemap_content, "xml")
urls = [loc.text for loc in soup.find_all("loc")]
print(f"Total {len(urls)} pages mile sitemap mein")


# # === Step 1: Sitemap se URLs fetch ===
# print("Sitemap se URLs fetch kar raha hoon...")
# headers = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"}
# response = requests.get(SITEMAP_URL, headers=headers)
# if response.status_code != 200:
#     print(f"Sitemap fetch failed: {response.status_code}")
#     exit()

# soup = BeautifulSoup(response.content, "xml")
# urls = [loc.text for loc in soup.find_all("loc")]
# print(f"Total {len(urls)} pages mile sitemap mein")

# if not urls:
#     print("Sitemap mein koi URL nahi mila. Exiting.")
#     exit()

# === Step 2: Har page ka text extract karo ===
# def fetch_page_text(url):
#     try:
#         resp = requests.get(url, timeout=10, headers=headers)
#         if resp.status_code != 200:
#             return ""
#         soup_page = BeautifulSoup(resp.content, "html.parser")
#         for script in soup_page(["script", "style", "nav", "header", "footer"]):
#             script.decompose()
#         text = soup_page.get_text(separator=" ", strip=True)
#         if len(text) < 200:
#             return ""
#         return text
#     except Exception as e:
#         print(f"Error fetching {url}: {e}")
#         return ""

# all_texts = []
# for i, url in enumerate(urls):
#     print(f"Fetching {i+1}/{len(urls)}: {url}")
#     text = fetch_page_text(url)
#     if text:
#         all_texts.append(text)
#     time.sleep(0.5)

# print(f"Total {len(all_texts)} pages ka proper text mila")

# if not all_texts:
#     print("Koi valid page text nahi mila. Exiting.")
#     exit()

# === Step 2: Har page ka text extract karo (headers free, error-free) ===
def fetch_page_text(url):
    try:
        # Simple request without headers
        resp = requests.get(url, timeout=10)
        if resp.status_code != 200:
            print(f"Skipping {url}, status code {resp.status_code}")
            return ""
        soup_page = BeautifulSoup(resp.content, "html.parser")
        
        # Remove scripts, styles, nav, header, footer
        for tag in soup_page(["script", "style", "nav", "header", "footer"]):
            tag.decompose()
        
        text = soup_page.get_text(separator=" ", strip=True)
        
        # Skip very small pages
        if len(text) < 200:
            return ""
        
        return text
    except Exception as e:
        print(f"Error fetching {url}: {e}")
        return ""

# Fetch text for all URLs
all_texts = []
for i, url in enumerate(urls):
    print(f"Fetching {i+1}/{len(urls)}: {url}")
    text = fetch_page_text(url)
    if text:
        all_texts.append(text)
    time.sleep(0.5)  # polite delay

print(f"Total {len(all_texts)} pages ka proper text mila")

# Exit if no valid text
if not all_texts:
    print("Koi valid page text nahi mila. Exiting.")
    exit()


# === Step 3: Text ko chunks mein todo ===
def chunk_text(text, chunk_size=1000, overlap=200):
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start = end - overlap
        if start >= len(text):
            break
    return chunks

all_chunks = []
for text in all_texts:
    all_chunks.extend(chunk_text(text))

print(f"Total {len(all_chunks)} chunks bana diye")

if not all_chunks:
    print("Koi chunks nahi bane. Exiting.")
    exit()

# === Step 4: Cohere se embeddings ===
print("Embeddings generate kar raha hoon (thoda time lagega)...")
co = cohere.Client(COHERE_API_KEY)
response = co.embed(
    model='embed-english-v3.0',
    texts=all_chunks,
    truncate='END'
)
embeddings = response.embeddings
print("Embeddings ban gaye!")

# === Step 5: Qdrant connect + collection ready ===
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

if client.collection_exists(COLLECTION_NAME):
    client.delete_collection(COLLECTION_NAME)
    print("Purani collection delete ki")

client.create_collection(
    collection_name=COLLECTION_NAME,
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
)
print("Nayi collection banayi")

# === Step 6: Upload ===
points = [
    PointStruct(
        id=idx,
        vector=embeddings[idx],
        payload={"text": all_chunks[idx], "source_url": urls[idx % len(urls)]}
    )
    for idx in range(len(all_chunks))
]

client.upsert(collection_name=COLLECTION_NAME, points=points)
print(f"Success! {len(all_chunks)} points upload ho gaye Qdrant mein.")
print("Ab dashboard kholo → rag_full_book → points count hundreds mein hona chahiye. Chatbot perfect kaam karega!")
