# Quickstart: RAG Integration with Cohere and Qdrant

## Prerequisites

- Python 3.10+
- uv package manager (`pip install uv` or install via other methods)
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Navigate to the backend directory:
```bash
cd backend
```

2. Create a virtual environment and install dependencies with uv:
```bash
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install cohere qdrant-client beautifulsoup4 requests lxml
```

3. Create your environment file:
```bash
cp .env.example .env
```

4. Edit `.env` and add your API keys:
```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Usage

1. Run the main script to execute the complete pipeline:
```bash
python main.py
```

The script will:
1. Fetch all URLs from the deployed site
2. Extract and clean text from each URL
3. Chunk the text appropriately
4. Generate embeddings using Cohere
5. Store embeddings in Qdrant with metadata

## Configuration

The main script can be configured via environment variables in your `.env` file:
- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `DEPLOYED_SITE_URL`: The URL of the deployed site to process (default: https://physical-ai-humanoid-robotic-book-ten.vercel.app/)

## Troubleshooting

- If you get rate limit errors, the script includes built-in delays to respect API limits
- Check that your API keys are valid and have sufficient quota
- Ensure your Qdrant cluster is accessible and the collection can be created
- Verify that the deployed site is accessible and contains the expected content