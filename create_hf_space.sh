#!/bin/bash

HF_TOKEN=$1
SPACE_NAME="physical-AI-Humanoid-Book"
USERNAME="Awais68"

echo "🚀 Creating Hugging Face Space..."
echo "📍 Space: $SPACE_NAME"

# Configure git
git config --global credential.helper store
echo "https://Awais68:$HF_TOKEN@huggingface.co" > ~/.git-credentials

# Add HF remote
git remote remove hf 2>/dev/null || true
git remote add hf https://huggingface.co/spaces/$USERNAME/$SPACE_NAME.git

# Push to HF Spaces
echo "📤 Pushing to Hugging Face Spaces..."
git push hf main --force

echo "✅ Deployment complete!"
echo "🌐 Space URL: https://huggingface.co/spaces/$USERNAME/$SPACE_NAME"
