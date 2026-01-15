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

SITEMAP_URL = "https://physical-ai-humanoid-robotic-book-ten.vercel.app/sitemap.xml"

# === Step 1: Sitemap se URLs fetch ===
print("Sitemap se URLs fetch kar raha hoon...")
headers = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"}
response = requests.get(SITEMAP_URL, headers=headers)
if response.status_code != 200:
    print(f"Sitemap fetch failed: {response.status_code}")
    exit()

soup = BeautifulSoup(response.content, "xml")
urls = [loc.text for loc in soup.find_all("loc")]
print(f"Total {len(urls)} pages mile sitemap mein")

if not urls:
    print("Sitemap mein koi URL nahi mila. Exiting.")
    exit()

# سب URLs کو correct domain پر convert کرو
new_urls = []
for url in urls:
    if url.startswith("https://physical-ai-robotics.dev"):
        new_url = url.replace("https://physical-ai-robotics.dev", "https://physical-ai-humanoid-robotic-book-ten.vercel.app")
        new_urls.append(new_url)
    else:
        new_urls.append(url)
urls = new_urls

print(f"Converted {len(urls)} URLs to correct domain")

# === Step 2: Har page ka text extract karo ===
def fetch_page_text(url):
    try:
        resp = requests.get(url, timeout=10, headers=headers)
        if resp.status_code != 200:
            return ""
        soup_page = BeautifulSoup(resp.content, "html.parser")
        for script in soup_page(["script", "style", "nav", "header", "footer"]):
            script.decompose()
        text = soup_page.get_text(separator=" ", strip=True)
        if len(text) < 200:
            return ""
        return text
    except Exception as e:
        print(f"Error fetching {url}: {e}")
        return ""

all_texts = []
for i, url in enumerate(urls):
    print(f"Fetching {i+1}/{len(urls)}: {url}")
    text = fetch_page_text(url)
    if text:
        all_texts.append(text)
    time.sleep(0.5)

print(f"Total {len(all_texts)} pages ka proper text mila")

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

# Manual batching to avoid rate limit
batch_size = 50
embeddings = []

print(f"Embeddings generate kar raha hoon {len(all_chunks)} chunks ke liye (batches mein)...")

for i in range(0, len(all_chunks), batch_size):
    batch = all_chunks[i:i + batch_size]
    
    response = co.embed(
        model='embed-english-v3.0',
        texts=batch,
        input_type='search_document',
        truncate='END'
    )
    
    embeddings.extend(response.embeddings)
    
    print(f"  Batch {i//batch_size + 1} complete ({len(batch)} chunks)")
    
    # Rate limit se bachne ke liye thoda wait
    time.sleep(5)  # 5 seconds wait har batch ke baad

print("Sab embeddings ban gaye!")

# === Step 5: Qdrant connect + collection ready (with high timeout) ===
from qdrant_client.models import VectorParams, Distance, OptimizersConfigDiff

client = QdrantClient(
    url=QDRANT_URL, 
    api_key=QDRANT_API_KEY,
    timeout=600  # 10 minutes timeout — bohot safe hai large uploads ke liye
)

if client.collection_exists(COLLECTION_NAME):
    client.delete_collection(COLLECTION_NAME)
    print("Purani collection delete ki")

# Collection banate waqt indexing temporarily off kar di (upload fast ho jayega)
client.create_collection(
    collection_name=COLLECTION_NAME,
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
    optimizers_config=OptimizersConfigDiff(
        indexing_threshold=0  # Indexing off during upload
    )
)
print("Nayi collection banayi (indexing off for fast upload)")

# === Step 6: Points banao ===
points = [
    PointStruct(
        id=idx,
        vector=embeddings[idx],
        payload={"text": all_chunks[idx], "source_url": urls[idx % len(urls)]}
    )
    for idx in range(len(all_chunks))
]

# === Step 7: Batches mein upload (128 points per batch) ===
upload_batch_size = 128  # Best size — fast aur timeout se safe
total_batches = (len(points) + upload_batch_size - 1) // upload_batch_size

print(f"Upload shuru kar raha hoon {len(points)} points ko {total_batches} batches mein...")

for i in range(0, len(points), upload_batch_size):
    batch = points[i:i + upload_batch_size]
    
    client.upsert(
        collection_name=COLLECTION_NAME,
        points=batch
    )
    
    print(f"  Batch {i//upload_batch_size + 1}/{total_batches} uploaded ({len(batch)} points)")

print(f"Success! {len(points)} points upload ho gaye Qdrant mein.")

# === Step 8: Indexing wapas ON kar do (must karna warna search slow rahega) ===
client.update_collection(
    collection_name=COLLECTION_NAME,
    optimizers_config=OptimizersConfigDiff(
        indexing_threshold=20000  # Default ya suitable value — indexing on
    )
)
print("Indexing wapas enable kar diya. Ab search fast hoga!")

print("Sab kuch complete! Ab dashboard kholo → rag_full_book → points count dekho. Chatbot ab perfect kaam karega! 🚀")