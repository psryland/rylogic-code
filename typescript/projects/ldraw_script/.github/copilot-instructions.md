# Copilot Instructions for LDraw Script VS Code Extension

## Overview

This is a VS Code language extension that provides syntax highlighting and IntelliSense auto-completion for LDraw Script (`.ldr` files).

## Build & Package

```powershell
# Install dependencies
npm install

# Build (parses templates + compiles TypeScript)
npm run build

# Package the extension into a .vsix file
npm run package
# Or use the publish script:
python publish.py
```

## Development

Press **F5** in VS Code to launch the Extension Development Host for testing.

## Architecture

| File | Purpose |
|------|---------|
| `src/extension.ts` | Extension entry point, registers CompletionItemProvider |
| `src/completionProvider.ts` | Context-aware completion logic |
| `src/templates.json` | Generated completion data (from build) |
| `scripts/parse-templates.ts` | Build-time parser for ldraw_templates.cpp |
| `syntaxes/ldr.tmLanguage.json` | TextMate grammar for syntax highlighting |
| `language-configuration.json` | Editor behaviors: comments, brackets, folding |
| `package.json` | Extension manifest |

## Template Source

The completion data is generated from:
```
../../../projects/rylogic/view3d-12/src/ldraw/ldraw_templates.cpp
```

`src/templates.json` is generated automatically before `npm run compile`, `npm run watch`, and VSIX packaging.
Run `npm run parse-templates` only when you need to refresh it without compiling.

## LDraw Script Syntax

The grammar highlights:
- **Keywords**: Tokens starting with `*` (e.g., `*Box`, `*Line`)
- **Preprocessor directives**: `#include`, `#define`, `#if`, `#ifdef`, etc.
- **Comments**: `//` line comments and `/* */` block comments
- **Strings**: Double-quoted strings with escape support
- **Numbers**: Integer literals

## Conventions

- Use TextMate scope naming conventions (e.g., `keyword.control`, `comment.line`, `string.quoted.double`)
- Test grammar changes in the Extension Development Host before packaging
- After modifying `ldraw_templates.cpp`, use the normal extension build/package commands; they regenerate completions automatically
