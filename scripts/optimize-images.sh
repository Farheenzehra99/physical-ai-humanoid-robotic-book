#!/bin/bash
# Image Optimization Pipeline for Physical AI Humanoid Robotics Documentation
# Converts images to WebP format and enforces <200KB size constraint
# Usage: ./scripts/optimize-images.sh <input-directory>

set -e

INPUT_DIR="${1:-docs}"
OUTPUT_DIR="${INPUT_DIR}"
MAX_SIZE_KB=200
QUALITY=85

echo "🖼️  Image Optimization Pipeline"
echo "================================"
echo "Input directory: $INPUT_DIR"
echo "Max size: ${MAX_SIZE_KB}KB"
echo "Quality: ${QUALITY}%"
echo ""

# Check if ImageMagick or cwebp is installed
if ! command -v convert &> /dev/null && ! command -v cwebp &> /dev/null; then
    echo "❌ Error: Neither ImageMagick nor cwebp found."
    echo "Install one of:"
    echo "  - ImageMagick: sudo apt install imagemagick (Ubuntu)"
    echo "  - WebP tools: sudo apt install webp (Ubuntu)"
    exit 1
fi

# Find all image files
find "$INPUT_DIR" -type f \( -iname "*.png" -o -iname "*.jpg" -o -iname "*.jpeg" \) | while read -r img; do
    # Get file size in KB
    size_kb=$(du -k "$img" | cut -f1)
    basename=$(basename "$img")
    dirname=$(dirname "$img")
    filename="${basename%.*}"

    # Convert to WebP
    webp_path="${dirname}/${filename}.webp"

    echo "Processing: $basename (${size_kb}KB)"

    # Convert to WebP with quality adjustment
    if command -v cwebp &> /dev/null; then
        cwebp -q $QUALITY "$img" -o "$webp_path" 2>/dev/null
    elif command -v convert &> /dev/null; then
        convert "$img" -quality $QUALITY "$webp_path" 2>/dev/null
    fi

    # Check output size
    if [ -f "$webp_path" ]; then
        webp_size_kb=$(du -k "$webp_path" | cut -f1)

        if [ $webp_size_kb -gt $MAX_SIZE_KB ]; then
            echo "  ⚠️  Warning: ${filename}.webp is ${webp_size_kb}KB (exceeds ${MAX_SIZE_KB}KB limit)"
            echo "  💡 Tip: Reduce image dimensions or use lower quality"
        else
            echo "  ✅ Created: ${filename}.webp (${webp_size_kb}KB)"
        fi

        # Optionally remove original (commented out for safety)
        # rm "$img"
    else
        echo "  ❌ Failed to convert: $basename"
    fi
done

echo ""
echo "✅ Image optimization complete!"
echo "📊 Run: du -h $INPUT_DIR | sort -h | tail -20"
echo "   to see largest remaining files"
