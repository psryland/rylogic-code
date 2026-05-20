# LDraw Script

Language support for LDraw script (`.ldr` files) with IntelliSense auto-completion.

## Features

- **Syntax highlighting** for LDraw script keywords, comments, strings, and numbers
- **Auto-completion** (Ctrl+Space) with context-aware keyword suggestions
  - Shows valid keywords based on cursor position
  - Displays full template syntax in the completion popup
  - Inserts only the keyword when accepted

## Usage

1. Open a `.ldr` file in VS Code
2. Press `Ctrl+Space` to trigger auto-completion
3. Select a keyword from the list to insert it

The extension provides hierarchical completions - only keywords valid in the current context are shown.

## Development

### Prerequisites

- Node.js (v18+)
- npm

### Build

```powershell
npm install
npm run build
```

This will:
1. Parse `ldraw_templates.cpp` to generate completion data
2. Compile TypeScript to JavaScript

The completion data is generated automatically before `compile`, `watch`, and VSIX packaging.

### Test

Press **F5** in VS Code to launch the Extension Development Host with the extension loaded.

### Package

```powershell
npm run package
# Or use the publish script:
python publish.py
```

The `.vsix` file will be created in the `publish/` directory.

## Publishing

This extension is published to the Visual Studio Marketplace under the **Rylogic** organisation.

## Release Notes

### 1.1.0

- Added IntelliSense auto-completion with context-aware suggestions
- Completions show template syntax but insert only the keyword

### 1.0.0

- Initial release with syntax highlighting

