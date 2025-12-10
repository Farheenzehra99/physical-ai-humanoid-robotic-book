# Data Model: Setting Up Isaac SDK

**Feature**: 001-isaac-sdk-setup
**Date**: 2025-12-09
**Phase**: Phase 1 (Design & Contracts)

This document defines the core entities and their relationships for the Isaac SDK setup documentation feature.

---

## Entity Definitions

### 1. DocumentationPage

Represents a single page of educational content in the Docusaurus site.

**Attributes**:
- `id`: string (unique identifier, e.g., "page-20-isaac-sdk-setup")
- `title`: string (display title)
- `module`: string (e.g., "Module 3: The AI-Robot Brain")
- `chapter`: string (e.g., "Chapter 7: NVIDIA Isaac Sim & SDK")
- `pageNumber`: integer (position in chapter, e.g., 20)
- `filePath`: string (relative path to MDX file)
- `description`: string (meta description for SEO)
- `tags`: string[] (searchable tags)
- `sidebarPosition`: integer (order in sidebar navigation)
- `estimatedReadTime`: integer (minutes)
- `createdDate`: ISO 8601 date string
- `lastModified`: ISO 8601 date string
- `author`: string
- `status`: enum ("draft" | "published" | "archived")

**Relationships**:
- Has many `InstallationStep`
- Has many `CodeSnippet`
- Has one `HardwareRequirementsTable`
- Has many `TroubleshootingEntry`

**Validation Rules**:
- `title` must be 10-100 characters
- `pageNumber` must be positive integer
- `tags` must have 2-10 items
- `estimatedReadTime` calculated from word count (200 words/minute)

**Example**:
```json
{
  "id": "page-20-isaac-sdk-setup",
  "title": "Setting Up Isaac SDK",
  "module": "Module 3: The AI-Robot Brain (NVIDIA Isaac™)",
  "chapter": "Chapter 7: NVIDIA Isaac Sim & SDK",
  "pageNumber": 20,
  "filePath": "docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/page-20-setting-up-isaac-sdk.mdx",
  "description": "Step-by-step guide to installing NVIDIA Isaac Sim for robotics development",
  "tags": ["isaac-sim", "installation", "setup", "gpu-requirements", "nvidia"],
  "sidebarPosition": 20,
  "estimatedReadTime": 8,
  "createdDate": "2025-12-09T00:00:00Z",
  "lastModified": "2025-12-09T00:00:00Z",
  "author": "Physical AI Curriculum Team",
  "status": "draft"
}
```

---

### 2. InstallationStep

Represents a single step in the installation procedure.

**Attributes**:
- `id`: string (unique identifier)
- `stepNumber`: integer (sequence number)
- `title`: string (short description, e.g., "Download Omniverse Launcher")
- `description`: string (detailed instructions in markdown)
- `platform`: enum ("ubuntu" | "windows" | "both")
- `commands`: TerminalCommand[] (shell commands to execute)
- `expectedOutput`: string (what user should see)
- `estimatedDuration`: integer (seconds)
- `prerequisites`: string[] (IDs of prerequisite steps)
- `isOptional`: boolean
- `warningMessage`: string | null (caution for users)

**Relationships**:
- Belongs to `DocumentationPage`
- Has many `TerminalCommand`
- References zero or more prerequisite `InstallationStep`

**Validation Rules**:
- `stepNumber` must be unique within page and sequential
- `title` must be 5-50 characters
- `description` must be 20-500 characters
- If `isOptional` is true, must have explanation in `description`

**Example**:
```json
{
  "id": "step-download-launcher",
  "stepNumber": 1,
  "title": "Download Omniverse Launcher",
  "description": "Visit the NVIDIA Isaac Sim download page and obtain the Omniverse Launcher installer for your operating system.",
  "platform": "both",
  "commands": [],
  "expectedOutput": "Installer file saved to Downloads folder",
  "estimatedDuration": 120,
  "prerequisites": [],
  "isOptional": false,
  "warningMessage": null
}
```

---

### 3. TerminalCommand

Represents a command to be executed in a terminal/shell.

**Attributes**:
- `id`: string (unique identifier)
- `command`: string (actual command text)
- `shell`: enum ("bash" | "powershell" | "sh")
- `platform`: enum ("ubuntu" | "windows" | "macos" | "all")
- `emojiPrefix`: string (visual indicator, e.g., "🔧" for setup)
- `description`: string (what the command does)
- `requiresSudo`: boolean
- `workingDirectory`: string | null
- `expectedReturnCode`: integer (default 0)
- `canFail`: boolean (if true, failure doesn't block next steps)

**Relationships**:
- Belongs to `InstallationStep`
- May reference environment variables

**Validation Rules**:
- `command` must not contain hardcoded secrets
- `emojiPrefix` must be one of approved set (🔧🚀📦✅🧪⬇️)
- If `requiresSudo`, must have security warning in parent step

**Emoji Convention**:
- 🔧 Setup/configuration
- 📦 Package installation
- ⬇️ Download
- 🚀 Launch/run
- ✅ Verification
- 🧪 Testing

**Example**:
```json
{
  "id": "cmd-install-launcher-ubuntu",
  "command": "chmod +x omniverse-launcher-linux.AppImage",
  "shell": "bash",
  "platform": "ubuntu",
  "emojiPrefix": "🔧",
  "description": "Make launcher executable",
  "requiresSudo": false,
  "workingDirectory": "~/Downloads",
  "expectedReturnCode": 0,
  "canFail": false
}
```

---

### 4. CodeSnippet

Represents executable code examples (primarily Python).

**Attributes**:
- `id`: string (unique identifier)
- `language`: enum ("python" | "bash" | "yaml" | "json" | "dockerfile")
- `title`: string (filename or description)
- `code`: string (actual source code)
- `showLineNumbers`: boolean
- `highlightedLines`: integer[] (lines to emphasize)
- `executionContext`: enum ("standalone" | "jupyter" | "repl" | "isaac-sim")
- `expectedOutput`: string
- `testCommand`: string (how to validate)
- `dependencies`: string[] (required packages)
- `purpose`: string (what this code demonstrates)

**Relationships**:
- Belongs to `DocumentationPage`
- May have associated test cases

**Validation Rules**:
- `code` must be syntactically valid for `language`
- `title` should end with appropriate extension (.py, .sh, .yaml, etc.)
- If `executionContext` is "isaac-sim", must import from `omni.isaac.*`
- `testCommand` must be executable and return expected result

**Example**:
```json
{
  "id": "snippet-verify-installation",
  "language": "python",
  "title": "test_installation.py",
  "code": "from omni.isaac.kit import SimulationApp\n\nsimulation_app = SimulationApp({\"headless\": True})\n\nfrom omni.isaac.core import World\n\nworld = World()\nprint(\"✅ Isaac Sim installed successfully!\")\n\nsimulation_app.close()",
  "showLineNumbers": true,
  "highlightedLines": [1, 5, 8],
  "executionContext": "isaac-sim",
  "expectedOutput": "✅ Isaac Sim installed successfully!",
  "testCommand": "python3 test_installation.py",
  "dependencies": ["omni.isaac.kit", "omni.isaac.core"],
  "purpose": "Verify Isaac Sim installation by creating a basic world"
}
```

---

### 5. HardwareRequirement

Represents hardware specifications for running Isaac Sim.

**Attributes**:
- `id`: string (unique identifier)
- `componentType`: enum ("gpu" | "cpu" | "ram" | "storage" | "os")
- `tier`: enum ("minimum" | "recommended" | "extreme")
- `specification`: string (human-readable spec)
- `value`: object (structured data for comparisons)
- `estimatedPrice`: number (USD, nullable for price volatility)
- `priceLastUpdated`: ISO 8601 date string
- `purchaseLinks`: PurchaseLink[]
- `notes`: string (additional context)

**Sub-Entity: value (GPU)**:
```typescript
{
  model: string,           // e.g., "NVIDIA RTX 4070 Ti"
  vram: number,            // GB
  computeCapability: number, // e.g., 8.9
  architecture: string     // e.g., "Ada Lovelace"
}
```

**Sub-Entity: PurchaseLink**:
```typescript
{
  retailer: string,  // e.g., "Amazon"
  url: string,       // actual purchase link
  verified: boolean, // link checked in last 30 days
  lastChecked: ISO 8601 date string
}
```

**Relationships**:
- Belongs to `HardwareRequirementsTable`
- Grouped by `tier`

**Validation Rules**:
- `estimatedPrice` must be positive number or null
- `priceLastUpdated` must be within 90 days for published docs
- `purchaseLinks` must have at least 1 verified link
- GPU `vram` must be >= 6GB for minimum tier

**Example**:
```json
{
  "id": "hw-gpu-recommended",
  "componentType": "gpu",
  "tier": "recommended",
  "specification": "NVIDIA RTX 4070 Ti (12GB VRAM)",
  "value": {
    "model": "NVIDIA RTX 4070 Ti",
    "vram": 12,
    "computeCapability": 8.9,
    "architecture": "Ada Lovelace"
  },
  "estimatedPrice": 849,
  "priceLastUpdated": "2025-12-09T00:00:00Z",
  "purchaseLinks": [
    {
      "retailer": "Amazon",
      "url": "https://amazon.com/...",
      "verified": true,
      "lastChecked": "2025-12-09T00:00:00Z"
    },
    {
      "retailer": "Newegg",
      "url": "https://newegg.com/...",
      "verified": true,
      "lastChecked": "2025-12-09T00:00:00Z"
    }
  ],
  "notes": "Best price-to-performance ratio for student projects"
}
```

---

### 6. HardwareRequirementsTable

Aggregates hardware requirements across tiers.

**Attributes**:
- `id`: string (unique identifier)
- `title`: string (e.g., "Hardware Requirements for Isaac Sim")
- `caption`: string (accessibility description)
- `lastUpdated`: ISO 8601 date string
- `requirements`: HardwareRequirement[] (grouped by tier)

**Relationships**:
- Belongs to `DocumentationPage`
- Has many `HardwareRequirement`

**Validation Rules**:
- Must have entries for all tiers (minimum, recommended, extreme)
- Must have entries for all component types (gpu, cpu, ram, storage, os)
- `lastUpdated` must be within 90 days for published pages

**Example**:
```json
{
  "id": "hw-table-isaac-sim",
  "title": "Hardware Requirements for Isaac Sim 2024.1",
  "caption": "Comparison of minimum, recommended, and extreme hardware specifications for running NVIDIA Isaac Sim",
  "lastUpdated": "2025-12-09T00:00:00Z",
  "requirements": [
    { "tier": "minimum", "componentType": "gpu", "..." },
    { "tier": "recommended", "componentType": "gpu", "..." },
    { "tier": "extreme", "componentType": "gpu", "..." }
  ]
}
```

---

### 7. TroubleshootingEntry

Represents a common problem and its solutions.

**Attributes**:
- `id`: string (unique identifier)
- `problem`: string (short title, e.g., "CUDA initialization failed")
- `cause`: string (explanation of root cause)
- `symptoms`: string[] (how user recognizes this issue)
- `solutions`: Solution[] (ordered list of remedies)
- `relatedErrors`: string[] (error messages this applies to)
- `frequency`: enum ("common" | "occasional" | "rare")
- `severity`: enum ("blocker" | "major" | "minor")

**Sub-Entity: Solution**:
```typescript
{
  stepNumber: number,
  description: string,
  commands: TerminalCommand[],
  expectedResult: string,
  isDefinitive: boolean  // true if this solves the problem, false if workaround
}
```

**Relationships**:
- Belongs to `DocumentationPage`
- May reference `InstallationStep` that commonly causes issue

**Validation Rules**:
- Must have at least 1 solution
- Solutions ordered by likelihood of success
- `problem` should be phrased as user experiences it

**Example**:
```json
{
  "id": "ts-cuda-init-failed",
  "problem": "CUDA initialization failed",
  "cause": "Outdated NVIDIA drivers or CUDA toolkit mismatch",
  "symptoms": [
    "Error message: \"RuntimeError: CUDA initialization failed\"",
    "nvidia-smi command not found",
    "GPU not detected in Isaac Sim logs"
  ],
  "solutions": [
    {
      "stepNumber": 1,
      "description": "Check current driver version",
      "commands": [
        {
          "command": "nvidia-smi",
          "shell": "bash",
          "platform": "ubuntu",
          "emojiPrefix": "🔍"
        }
      ],
      "expectedResult": "Driver version 525.60.13 or higher",
      "isDefinitive": false
    },
    {
      "stepNumber": 2,
      "description": "Update NVIDIA drivers (Ubuntu)",
      "commands": [
        {
          "command": "sudo apt update && sudo apt install nvidia-driver-535",
          "shell": "bash",
          "platform": "ubuntu",
          "emojiPrefix": "📦"
        }
      ],
      "expectedResult": "Drivers updated, reboot required",
      "isDefinitive": true
    }
  ],
  "relatedErrors": ["RuntimeError: CUDA initialization failed", "CUDA_ERROR_NO_DEVICE"],
  "frequency": "common",
  "severity": "blocker"
}
```

---

## Entity Relationships Diagram

```
DocumentationPage
├── InstallationStep (1:N)
│   └── TerminalCommand (1:N)
├── CodeSnippet (1:N)
├── HardwareRequirementsTable (1:1)
│   └── HardwareRequirement (1:N)
│       └── PurchaseLink (1:N)
└── TroubleshootingEntry (1:N)
    └── Solution (1:N)
        └── TerminalCommand (1:N)
```

---

## State Transitions

### DocumentationPage Status Flow

```
draft → published → archived
  ↓         ↓
  └────────┘ (can revert to draft for major updates)
```

**Transition Rules**:
- `draft → published`: Requires all validation checks to pass
  - All `InstallationStep` tested on target platforms
  - All `CodeSnippet` executed successfully
  - All `HardwareRequirement.purchaseLinks` verified within 30 days
  - Lighthouse score >95 (accessibility)
  - Zero broken links
- `published → draft`: Major content revisions needed
- `published → archived`: Content outdated (e.g., Isaac Sim version deprecated)

---

## Validation Constraints

### Cross-Entity Constraints

1. **Installation Step Sequence**:
   - `InstallationStep.stepNumber` must be sequential (1, 2, 3, ..., N)
   - No gaps allowed
   - Prerequisites must reference earlier steps only

2. **Platform Consistency**:
   - If `InstallationStep.platform` is "ubuntu", all child `TerminalCommand.platform` must be "ubuntu" or "all"
   - Similar rule for Windows

3. **Hardware Tier Logic**:
   - For each `componentType`, `minimum.value` < `recommended.value` < `extreme.value`
   - Example: minimum VRAM (6GB) < recommended VRAM (12GB) < extreme VRAM (24GB)

4. **Code Snippet Dependencies**:
   - All items in `CodeSnippet.dependencies` must be installable via pip or included in Isaac Sim
   - Must not reference deprecated APIs

5. **Price Staleness**:
   - If `HardwareRequirement.estimatedPrice` is not null, `priceLastUpdated` must be within 90 days
   - If older, price must be set to null and note added

---

## Usage Examples

### Creating a Complete Installation Guide

```json
{
  "page": {
    "id": "page-20-isaac-sdk-setup",
    "title": "Setting Up Isaac SDK",
    "status": "draft"
  },
  "steps": [
    {
      "stepNumber": 1,
      "title": "Download Omniverse Launcher",
      "platform": "both",
      "commands": []
    },
    {
      "stepNumber": 2,
      "title": "Install Omniverse Launcher",
      "platform": "ubuntu",
      "commands": [
        {
          "command": "chmod +x omniverse-launcher-linux.AppImage",
          "emojiPrefix": "🔧"
        },
        {
          "command": "./omniverse-launcher-linux.AppImage",
          "emojiPrefix": "🚀"
        }
      ]
    }
  ],
  "codeSnippets": [
    {
      "title": "test_installation.py",
      "language": "python",
      "code": "from omni.isaac.kit import SimulationApp\n...",
      "executionContext": "isaac-sim"
    }
  ],
  "hardwareTable": {
    "title": "Hardware Requirements for Isaac Sim 2024.1",
    "requirements": [
      {
        "tier": "minimum",
        "componentType": "gpu",
        "specification": "NVIDIA GTX 1060 (6GB VRAM)"
      },
      {
        "tier": "recommended",
        "componentType": "gpu",
        "specification": "NVIDIA RTX 4070 Ti (12GB VRAM)"
      }
    ]
  },
  "troubleshooting": [
    {
      "problem": "CUDA initialization failed",
      "solutions": [
        {
          "description": "Update NVIDIA drivers",
          "isDefinitive": true
        }
      ]
    }
  ]
}
```

---

## Implementation Notes

### Storage Strategy

**For MVP (Static Site)**:
- Entities embedded directly in MDX front matter and content
- No database required
- Validation via JSON Schema in CI/CD

**For Future (Dynamic Site)**:
- Store in PostgreSQL or MongoDB
- API for hardware price updates
- Admin UI for content management

### JSON Schema Validation

All entities will have corresponding JSON Schema definitions in `contracts/` directory for automated validation during CI/CD.

---

## Next Steps

1. Generate JSON Schema files for each entity (Phase 1: contracts/)
2. Create quickstart.md using these entities
3. Update agent context with entity definitions
4. Validate against constitution principles

This data model ensures structured, maintainable, and testable documentation content.
