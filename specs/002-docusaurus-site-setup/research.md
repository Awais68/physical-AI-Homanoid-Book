# Research Findings: Docusaurus Static Site Setup

## Technology Choice: Docusaurus v3

**Decision**: Docusaurus v3 (latest stable) will be used as the static site generator.

**Rationale**: Docusaurus is specifically designed for documentation websites, offering built-in features for content organization (sidebar, versioning), search (Algolia integration), code highlighting, and a React-based component system for extensibility. Version 3 provides the latest features and optimizations.

**Alternatives Considered**:
- **Next.js/Gatsby**: While powerful for general-purpose React applications, they require more manual setup for documentation-specific features like sidebars, search, and versioning.
- **Jekyll/Hugo**: Simpler static site generators, but lack the integrated React/MDX capabilities and extensive plugin ecosystem of Docusaurus, making interactive content harder to implement.

## Best Practices for Docusaurus v3 Implementation

### 1. Content Structure (`docs/`, `src/`, `static/`)

- **Decision**: Adhere to the standard Docusaurus content structure.
- **Rationale**: This structure is optimized for Docusaurus's build process and leverages its conventions for automatic routing and asset management. Deviating would complicate maintenance and upgrades.
- `docs/`: Markdown (`.md` or `.mdx`) files for textbook content, organized into subdirectories for modules and weeks.
- `src/`: Custom React components (e.g., interactive exercises, custom admonitions) to be used within MDX files. This keeps custom logic separate from core Docusaurus files.
- `static/`: Non-processed assets like images, diagrams, and PDFs. These are copied directly to the build output.

### 2. Customization and Styling

- **Decision**: Utilize Docusaurus theming capabilities with custom CSS.
- **Rationale**: Docusaurus allows shadowing components for deep customization, and custom CSS provides granular control over styling while inheriting base theme properties. This ensures brand consistency and accessibility.
- Implement a `custom.css` file to override or add styles.
- Configure `docusaurus.config.js` for dark/light theme support.

### 3. Search Functionality (Algolia DocSearch)

- **Decision**: Integrate Algolia DocSearch as the primary search solution.
- **Rationale**: Algolia DocSearch is specifically designed for documentation, offering fast, relevant search results and easy integration with Docusaurus. It provides a superior user experience compared to client-side-only search.
- Requires an Algolia account and application ID/API key. These should be managed as environment variables, not hardcoded.

### 4. Code Syntax Highlighting (Prism) and Diagrams (Mermaid)

- **Decision**: Leverage Docusaurus's built-in support for Prism and integrate Mermaid.
- **Rationale**: Prism is the default and highly configurable syntax highlighter in Docusaurus. Mermaid is a popular tool for generating diagrams from text, fitting well with a version-control-friendly approach.
- Prism configuration can be done via `docusaurus.config.js`.
- Mermaid integration typically involves a Docusaurus plugin or custom MDX component.

### 5. Deployment (GitHub Pages & GitHub Actions)

- **Decision**: Deploy to GitHub Pages via GitHub Actions for automated CI/CD.
- **Rationale**: This provides a free, robust hosting solution tightly integrated with Git. GitHub Actions automate the build and deployment process, ensuring that every push to the main branch triggers an update, simplifying maintenance.
- The workflow should include steps for installing dependencies, building the Docusaurus site, and deploying the `build` output to GitHub Pages.

### 6. Navigation and Content Organization

- **Decision**: Configure sidebar navigation, breadcrumbs, and previous/next links as per Docusaurus best practices.
- **Rationale**: These features are crucial for user experience in a textbook-like environment, guiding students through modules and allowing them to understand their current location in the content hierarchy.
- `sidebars.js` will define the structure for module and week-based navigation.
- Automatic breadcrumbs and previous/next links are enabled by default in Docusaurus themes.

## Unresolved Questions / Areas for Further Consideration

- Specific styling guidelines (e.g., brand colors, font choices) for custom CSS.
- Detailed content organization plan for modules and weeks (e.g., how many sub-sections per week).
- Decision on whether to use a custom Docusaurus theme or stick to the default with CSS overrides.
- Strategy for managing and displaying different code examples (Python, C++) using tabs – whether to use Docusaurus tabs or a custom React component.
