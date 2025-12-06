# Quickstart Guide: Docusaurus Static Site Setup

This guide provides a quick overview of how to get started with the Docusaurus static site. It covers project initialization, dependency installation, running the development server, and building the site for deployment.

## 1. Prerequisites

Ensure you have Node.js (latest LTS) and npm/yarn installed on your system.

## 2. Project Initialization

To create a new Docusaurus project, navigate to your desired directory and run:

```bash
npx create-docusaurus@latest my-website classic --typescript
```

Replace `my-website` with your preferred project name. The `classic` template is a good starting point, and `--typescript` is recommended for React components.

## 3. Install Dependencies

Navigate into your new project directory and install the necessary dependencies:

```bash
cd my-website
npm install # or yarn install
```

## 4. Start Development Server

To run the site locally in development mode:

```bash
npm start # or yarn start
```

This will open your browser to `http://localhost:3000` (or another available port).

## 5. Build the Site

To build the static site for production, which generates the content in the `build/` directory:

```bash
npm run build # or yarn build
```

## 6. Add Content

- **Documentation**: Add Markdown (`.md`) or MDX (`.mdx`) files in the `docs/` directory. Organize them into subfolders for modules and weeks as specified.
- **Custom React Components**: Place your custom React components in the `src/` directory and import them into your MDX files.
- **Static Assets**: Put images, diagrams, and other static files in the `static/` directory.

For more detailed information, refer to the official [Docusaurus documentation](https://docusaurus.io/docs).
