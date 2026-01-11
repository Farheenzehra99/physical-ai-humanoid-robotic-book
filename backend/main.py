"""
Main entry point for Hugging Face Space deployment.

This file serves as the entry point for the FastAPI application
when deployed on Hugging Face Spaces.
"""

import os
import uvicorn
from src.api.app import app

if __name__ == "__main__":
    # For Hugging Face Spaces, use port 7860 as default
    port = int(os.environ.get("PORT", 7860))
    uvicorn.run(app, host="0.0.0.0", port=port)