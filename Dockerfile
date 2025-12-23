# Use the official Python image from the Docker Hub
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# Set work directory
WORKDIR /app

# Install system dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        postgresql-client \
        build-essential \
        libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY backend/requirements.txt /app/requirements.txt

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy project
COPY backend/ /app/

# Copy scripts
COPY scripts/ /app/scripts/


# Expose port
EXPOSE 7860

# Run the application
CMD ["uvicorn", "src.api.app:app", "--host", "0.0.0.0", "--port", "7860"]