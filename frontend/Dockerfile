# Multi-stage build for Docusaurus frontend
FROM node:18-alpine AS builder

WORKDIR /app

# Copy package files
COPY package.json package-lock.json ./

# Install dependencies
RUN npm ci

# Copy source code
COPY . .

# Build the site
RUN npm run build

# Production stage - Use Nginx to serve the static files
FROM nginx:alpine AS production

# Copy nginx configuration
RUN rm /etc/nginx/conf.d/default.conf
COPY nginx.conf /etc/nginx/conf.d/

# Copy built files from builder
COPY --from=builder /app/build /usr/share/nginx/html

# Expose port
EXPOSE 80

# Health check
HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
    CMD wget --quiet --tries=1 --spider http://localhost/health || exit 1

# Start Nginx
CMD ["nginx", "-g", "daemon off;"]
