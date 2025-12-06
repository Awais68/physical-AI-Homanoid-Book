# GitHub Actions CI/CD Configuration

This document describes the automated CI/CD pipelines configured for the Physical AI & Humanoid Robotics project.

## Workflows Overview

### 1. **Build & Test** (`build.yml`)

- **Trigger:** Push to main/develop/feature branches, Pull Requests
- **Node Version:** 20.x
- **Tasks:**
  - ✓ Checkout code
  - ✓ Install dependencies (with caching)
  - ✓ Run ESLint
  - ✓ TypeScript type checking
  - ✓ Code formatting validation
  - ✓ Build Docusaurus site
  - ✓ Upload build artifacts

**Runs on:** Every push and PR

---

### 2. **Deploy to GitHub Pages** (`deploy.yml`)

- **Trigger:** Push to main branch only
- **Permissions:** Pages write access
- **Tasks:**
  - ✓ Checkout code with full history
  - ✓ Install dependencies
  - ✓ Build documentation
  - ✓ Configure GitHub Pages
  - ✓ Deploy to GitHub Pages
  - ✓ Create deployment record

**Runs on:** Main branch commits (automatic deployment)

---

### 3. **Code Quality & Security** (`quality.yml`)

- **Trigger:** Push to main/develop/feature, PRs, Weekly schedule (Sundays 2 AM)
- **Jobs:**
  - **Security Scan:** `npm audit` dependency vulnerability check
  - **Dependency Check:** Lists outdated packages
  - **Code Quality:** ESLint, Prettier, TypeScript checks

**Runs on:** All commits and scheduled weekly

---

### 4. **Documentation Checks** (`documentation.yml`)

- **Trigger:** Changes to docs or sidebar configuration
- **Tasks:**
  - ✓ Build documentation
  - ✓ Validate sidebar structure
  - ✓ Check documentation directory
  - ✓ Count documentation files

**Runs on:** Documentation changes only

---

## Setup Instructions

### Prerequisites

1. GitHub repository created ✓
2. Repository has `main` branch
3. Enable GitHub Pages in repository settings
4. Node.js 20.x available

### Configuration Steps

#### Step 1: Enable GitHub Pages

1. Go to repository Settings → Pages
2. Set Source: "GitHub Actions"
3. Save

#### Step 2: Verify Branch Protection (Optional)

1. Go to Settings → Branches
2. Add branch protection rule for `main`:
   - Require status checks to pass before merging
   - Select: `build`, `deploy`

#### Step 3: Set Up Secrets (Optional)

No additional secrets required for basic setup, but you can add:

- `GITHUB_TOKEN` (auto-provided by GitHub)
- `NPM_TOKEN` (if using private npm packages)

---

## Workflow Details

### Build Pipeline

```
Code Push/PR
    ↓
Checkout → Install → Lint → TypeCheck → Format → Build → Upload Artifacts
    ↓
Pass/Fail Status
```

### Deploy Pipeline

```
Push to main
    ↓
Checkout → Install → Build → Upload to Pages → Deploy
    ↓
Live on GitHub Pages
```

---

## Status Checks

Each workflow has a status badge. Add to README.md:

```markdown
![Build & Test](https://github.com/Awais68/physical-AI-Homanoid-Book/actions/workflows/build.yml/badge.svg)
![Deploy](https://github.com/Awais68/physical-AI-Homanoid-Book/actions/workflows/deploy.yml/badge.svg)
![Code Quality](https://github.com/Awais68/physical-AI-Homanoid-Book/actions/workflows/quality.yml/badge.svg)
```

---

## Files Created

```
.github/
└── workflows/
    ├── build.yml              # Main CI pipeline
    ├── deploy.yml             # Deployment to GitHub Pages
    ├── quality.yml            # Code quality & security
    └── documentation.yml      # Documentation validation
```

---

## Running Workflows

### Automatic Triggers

- `build.yml`: Every push and PR
- `deploy.yml`: Push to main only
- `quality.yml`: Every push and weekly schedule
- `documentation.yml`: Documentation changes

### Manual Trigger

All workflows can be manually triggered from GitHub Actions tab:

1. Go to Actions → Select workflow
2. Click "Run workflow"
3. Select branch → Run

---

## Monitoring

### View Workflow Status

1. Go to GitHub repository
2. Click "Actions" tab
3. Select workflow to view details

### Failed Workflow Debugging

1. Click on the failed run
2. Expand failed job
3. Check error logs
4. Common issues:
   - Missing dependencies
   - TypeScript errors
   - Linting failures
   - Build errors

---

## Environment Variables

Add to workflow if needed:

```yaml
env:
  NODE_ENV: production
  CI: true
```

---

## Performance Optimization

### Caching Strategy

- npm dependencies cached: ~30s faster builds
- GitHub Actions runner cache: Reused across runs
- Artifact retention: 30 days (adjustable)

### Execution Time

- Build: ~60-90 seconds
- Deploy: ~120-150 seconds
- Quality checks: ~45-60 seconds

---

## Troubleshooting

### Issue: "Cannot find module"

**Solution:** Run `npm ci` in workflow (already configured)

### Issue: "Port already in use"

**Solution:** Tests don't start server, safe to ignore

### Issue: "Build artifacts not found"

**Solution:** Check build.yml output directory configuration

### Issue: "Deployment failed"

**Solution:**

1. Verify GitHub Pages enabled
2. Check branch protection rules
3. Ensure build succeeds first

---

## Next Steps

1. ✓ Workflows configured
2. Push to GitHub to activate
3. Monitor Actions tab
4. Add badges to README
5. Enable branch protection (optional)
6. Add notification alerts (optional)

---

## Additional Resources

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Docusaurus Deployment Guide](https://docusaurus.io/docs/deployment)
- [Node.js Actions](https://github.com/actions/setup-node)

---

## Support

For issues with CI/CD:

1. Check GitHub Actions tab for error details
2. Review workflow YAML syntax
3. Verify dependencies in package.json
4. Check Node.js version compatibility
