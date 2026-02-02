#!/bin/bash
# Deploy to Hugging Face Spaces
# Usage: ./deploy_hf.sh <HF_TOKEN>

set -e

HF_TOKEN=${1:-$HF_TOKEN}
HF_REPO_ID="Awais68/physical-AI-Humanoid-Book"
HF_REPO_TYPE="space"

if [ -z "$HF_TOKEN" ]; then
    echo "❌ Error: HF_TOKEN not provided"
    echo "Usage: ./deploy_hf.sh <HF_TOKEN>"
    exit 1
fi

echo "🚀 Starting Hugging Face Space deployment..."
echo "📦 Repository: $HF_REPO_ID"
echo "🔧 Type: $HF_REPO_TYPE"

# Create space if not exists
echo -e "\n[1] Creating/Checking HF Space..."
huggingface-cli repo create \
    --repo_id="$HF_REPO_ID" \
    --type="$HF_REPO_TYPE" \
    --private=False \
    --space_sdk=docker \
    --token="$HF_TOKEN" 2>/dev/null || echo "✓ Space already exists"

# Add HF remote
echo -e "\n[2] Adding Hugging Face remote..."
git remote remove hf 2>/dev/null || true
git remote add hf https://huggingface.co/spaces/$HF_REPO_ID

# Configure git for HF
echo -e "\n[3] Configuring git credentials..."
export GIT_AUTHOR_NAME="Physical AI Bot"
export GIT_AUTHOR_EMAIL="bot@physical-ai.local"
export GIT_COMMITTER_NAME="Physical AI Bot"
export GIT_COMMITTER_EMAIL="bot@physical-ai.local"

# Push to Hugging Face
echo -e "\n[4] Pushing to Hugging Face..."
git push https://oauth2:$HF_TOKEN@huggingface.co/spaces/$HF_REPO_ID main --force

echo -e "\n✅ Deployment Complete!"
echo "🌐 Space URL: https://huggingface.co/spaces/$HF_REPO_ID"
echo "📝 Logs: huggingface.co/spaces/$HF_REPO_ID/logs"
