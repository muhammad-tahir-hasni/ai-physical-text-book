#!/bin/bash
# Project Cleanup Script
# Generated: 2025-12-16
# Purpose: Clean up unnecessary files and prepare project for development

set -e  # Exit on error

echo "================================="
echo "Project Cleanup Script"
echo "================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Step 1: Clean build artifacts
echo "Step 1: Cleaning build artifacts..."

if [ -d "frontend/.docusaurus" ]; then
    rm -rf frontend/.docusaurus
    print_status "Removed frontend/.docusaurus"
fi

if [ -d "robotic/.docusaurus" ]; then
    rm -rf robotic/.docusaurus
    print_status "Removed robotic/.docusaurus"
fi

if [ -d "frontend/build" ]; then
    rm -rf frontend/build
    print_status "Removed frontend/build"
fi

if [ -d "robotic/build" ]; then
    rm -rf robotic/build
    print_status "Removed robotic/build"
fi

echo ""

# Step 2: Clean Python cache
echo "Step 2: Cleaning Python cache files..."

find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find . -type f -name "*.pyc" -delete 2>/dev/null || true
find . -type f -name "*.pyo" -delete 2>/dev/null || true
find . -type f -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true

print_status "Removed Python cache files"
echo ""

# Step 3: Clean macOS files
echo "Step 3: Cleaning macOS system files..."

find . -name ".DS_Store" -type f -delete 2>/dev/null || true
print_status "Removed .DS_Store files"
echo ""

# Step 4: Verify critical files are in place
echo "Step 4: Verifying project structure..."

if [ -f ".gitignore" ]; then
    print_status ".gitignore present"
else
    print_error ".gitignore missing!"
fi

if [ -f "backend/.env.example" ]; then
    print_status "backend/.env.example present"
else
    print_warning "backend/.env.example missing"
fi

if [ -f "frontend/.env.example" ]; then
    print_status "frontend/.env.example present"
else
    print_warning "frontend/.env.example missing"
fi

if [ -f "backend/requirements.txt" ]; then
    print_status "backend/requirements.txt present"
else
    print_error "backend/requirements.txt missing!"
fi

if [ -f "frontend/package.json" ]; then
    print_status "frontend/package.json present"
else
    print_error "frontend/package.json missing!"
fi

echo ""

# Step 5: Check for sensitive files
echo "Step 5: Checking for sensitive files that shouldn't be committed..."

SENSITIVE_FOUND=0

if [ -f ".env" ] && grep -q "sk-ant-\|sk-proj-" .env 2>/dev/null; then
    print_error "Root .env contains API keys!"
    SENSITIVE_FOUND=1
fi

if [ -f "backend/.env" ] && grep -q "sk-ant-\|sk-proj-" backend/.env 2>/dev/null; then
    print_error "backend/.env contains API keys!"
    SENSITIVE_FOUND=1
fi

if [ -f "frontend/.env" ] && grep -q "sk-ant-\|sk-proj-" frontend/.env 2>/dev/null; then
    print_error "frontend/.env contains API keys!"
    SENSITIVE_FOUND=1
fi

if [ $SENSITIVE_FOUND -eq 0 ]; then
    print_status "No sensitive data found in tracked files"
else
    print_warning "Sensitive data detected! Review .env files before committing."
fi

echo ""

# Step 6: Size report
echo "Step 6: Disk usage report..."

if [ -d "frontend/node_modules" ]; then
    FRONTEND_NODE_SIZE=$(du -sh frontend/node_modules 2>/dev/null | cut -f1)
    echo "  frontend/node_modules: $FRONTEND_NODE_SIZE"
fi

if [ -d "robotic/node_modules" ]; then
    ROBOTIC_NODE_SIZE=$(du -sh robotic/node_modules 2>/dev/null | cut -f1)
    echo "  robotic/node_modules: $ROBOTIC_NODE_SIZE"
fi

if [ -d "backend/venv" ]; then
    BACKEND_VENV_SIZE=$(du -sh backend/venv 2>/dev/null | cut -f1)
    echo "  backend/venv: $BACKEND_VENV_SIZE"
fi

echo ""

# Step 7: Git status
echo "Step 7: Git status..."

CHANGED_FILES=$(git status --porcelain | wc -l | tr -d ' ')
echo "  Files changed/staged: $CHANGED_FILES"

if [ $CHANGED_FILES -gt 0 ]; then
    print_warning "Uncommitted changes detected"
    echo ""
    echo "Modified files:"
    git status --short | head -20
    if [ $CHANGED_FILES -gt 20 ]; then
        echo "  ... and $((CHANGED_FILES - 20)) more"
    fi
else
    print_status "Working directory clean"
fi

echo ""

# Summary
echo "================================="
echo "Cleanup Summary"
echo "================================="
echo ""
print_status "Build artifacts cleaned"
print_status "Python cache removed"
print_status "macOS files removed"

if [ $SENSITIVE_FOUND -eq 0 ]; then
    print_status "No sensitive data in tracked files"
else
    print_error "Sensitive data needs attention"
fi

echo ""
echo "Next steps:"
echo "  1. Review PROJECT_ANALYSIS.md for detailed findings"
echo "  2. Set up environment variables (cp .env.example .env)"
echo "  3. Install dependencies:"
echo "     - Backend: cd backend && pip install -r requirements.txt"
echo "     - Frontend: cd frontend && npm install"
echo "  4. Run git status to review changes"
echo "  5. Commit security fixes: git commit -m 'Security: Sanitize .env files and update .gitignore'"
echo ""
echo "================================="
echo "Cleanup Complete!"
echo "================================="
