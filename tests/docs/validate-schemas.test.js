/**
 * JSON Schema Validation Test Harness
 * Validates documentation entities against JSON schemas
 *
 * Tests:
 * - DocumentationPage schema
 * - CodeSnippet schema
 * - HardwareRequirement schema
 * - InstallationStep schema
 * - TroubleshootingEntry schema
 */

const Ajv = require('ajv');
const fs = require('fs');
const path = require('path');

// Initialize AJV with strict mode
const ajv = new Ajv({ allErrors: true, strict: true });

// Load JSON schemas
const schemasDir = path.join(__dirname, '../../specs/001-isaac-sdk-setup/contracts');
const schemas = {
  documentationPage: require(path.join(schemasDir, 'documentation-page.schema.json')),
  codeSnippet: require(path.join(schemasDir, 'code-snippet.schema.json')),
  hardwareRequirement: require(path.join(schemasDir, 'hardware-requirement.schema.json')),
  installationStep: require(path.join(schemasDir, 'installation-step.schema.json')),
  troubleshootingEntry: require(path.join(schemasDir, 'troubleshooting-entry.schema.json')),
};

// Compile schemas
const validators = {};
Object.keys(schemas).forEach(key => {
  validators[key] = ajv.compile(schemas[key]);
});

describe('JSON Schema Validation', () => {
  describe('DocumentationPage Schema', () => {
    it('should validate a valid documentation page', () => {
      const validPage = {
        id: 'page-20-isaac-sdk-setup',
        title: 'Setting Up Isaac SDK',
        module: 'Module 3: The AI-Robot Brain',
        chapter: 'Chapter 7: NVIDIA Isaac Sim & SDK',
        pageNumber: 20,
        filePath: 'docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/page-20-setting-up-isaac-sdk.mdx',
        description: 'Step-by-step guide to installing NVIDIA Isaac Sim for robotics development',
        tags: ['isaac-sim', 'installation', 'setup'],
        status: 'draft',
      };

      const valid = validators.documentationPage(validPage);
      if (!valid) {
        console.error('Validation errors:', validators.documentationPage.errors);
      }
      expect(valid).toBe(true);
    });

    it('should reject invalid page ID format', () => {
      const invalidPage = {
        id: 'invalid-id-format', // Missing page- prefix
        title: 'Test Page',
        module: 'Test Module',
        chapter: 'Test Chapter',
        pageNumber: 1,
        filePath: 'docs/test.mdx',
        description: 'Test description for validation',
        tags: ['test'],
        status: 'draft',
      };

      const valid = validators.documentationPage(invalidPage);
      expect(valid).toBe(false);
    });
  });

  describe('CodeSnippet Schema', () => {
    it('should validate a valid code snippet', () => {
      const validSnippet = {
        id: 'snippet-test-installation',
        language: 'python',
        title: 'test_installation.py',
        code: 'from omni.isaac.kit import SimulationApp\nprint("Hello")',
        executionContext: 'isaac-sim',
        purpose: 'Verify Isaac Sim installation',
      };

      const valid = validators.codeSnippet(validSnippet);
      if (!valid) {
        console.error('Validation errors:', validators.codeSnippet.errors);
      }
      expect(valid).toBe(true);
    });

    it('should reject unsupported language', () => {
      const invalidSnippet = {
        id: 'snippet-test',
        language: 'ruby', // Not in enum
        title: 'test.rb',
        code: 'puts "Hello"',
        executionContext: 'standalone',
        purpose: 'Test purpose',
      };

      const valid = validators.codeSnippet(invalidSnippet);
      expect(valid).toBe(false);
    });
  });

  describe('HardwareRequirement Schema', () => {
    it('should validate a valid GPU requirement', () => {
      const validGPU = {
        id: 'hw-gpu-recommended',
        componentType: 'gpu',
        tier: 'recommended',
        specification: 'NVIDIA RTX 4070 Ti (12GB VRAM)',
        value: {
          model: 'NVIDIA RTX 4070 Ti',
          vram: 12,
          computeCapability: 8.9,
          architecture: 'Ada Lovelace',
        },
      };

      const valid = validators.hardwareRequirement(validGPU);
      if (!valid) {
        console.error('Validation errors:', validators.hardwareRequirement.errors);
      }
      expect(valid).toBe(true);
    });

    it('should reject insufficient VRAM', () => {
      const invalidGPU = {
        id: 'hw-gpu-invalid',
        componentType: 'gpu',
        tier: 'minimum',
        specification: 'GTX 960 (4GB VRAM)',
        value: {
          model: 'GTX 960',
          vram: 4, // Below minimum 6GB
          computeCapability: 5.2,
          architecture: 'Maxwell',
        },
      };

      const valid = validators.hardwareRequirement(invalidGPU);
      expect(valid).toBe(false);
    });
  });

  describe('InstallationStep Schema', () => {
    it('should validate a valid installation step', () => {
      const validStep = {
        id: 'step-download-launcher',
        stepNumber: 1,
        title: 'Download Omniverse Launcher',
        description: 'Visit the NVIDIA Isaac Sim download page and obtain the launcher',
        platform: 'both',
      };

      const valid = validators.installationStep(validStep);
      if (!valid) {
        console.error('Validation errors:', validators.installationStep.errors);
      }
      expect(valid).toBe(true);
    });

    it('should reject invalid step number', () => {
      const invalidStep = {
        id: 'step-invalid',
        stepNumber: 0, // Must be >= 1
        title: 'Invalid Step',
        description: 'This step has an invalid number',
        platform: 'ubuntu',
      };

      const valid = validators.installationStep(invalidStep);
      expect(valid).toBe(false);
    });
  });

  describe('TroubleshootingEntry Schema', () => {
    it('should validate a valid troubleshooting entry', () => {
      const validEntry = {
        id: 'ts-cuda-init-failed',
        problem: 'CUDA initialization failed',
        cause: 'Outdated NVIDIA drivers',
        symptoms: ['Error message: RuntimeError'],
        solutions: [
          {
            stepNumber: 1,
            description: 'Update NVIDIA drivers',
            expectedResult: 'Driver version 525.60.13+',
            isDefinitive: true,
          },
        ],
        frequency: 'common',
        severity: 'blocker',
      };

      const valid = validators.troubleshootingEntry(validEntry);
      if (!valid) {
        console.error('Validation errors:', validators.troubleshootingEntry.errors);
      }
      expect(valid).toBe(true);
    });

    it('should reject empty solutions array', () => {
      const invalidEntry = {
        id: 'ts-invalid',
        problem: 'Test Problem',
        cause: 'Test cause',
        symptoms: ['Symptom 1'],
        solutions: [], // Must have at least 1 solution
        frequency: 'rare',
        severity: 'minor',
      };

      const valid = validators.troubleshootingEntry(invalidEntry);
      expect(valid).toBe(false);
    });
  });
});

describe('Schema Integration Tests', () => {
  it('should validate complete documentation page with all entities', () => {
    // This would validate a full page with embedded entities
    // Placeholder for future integration test
    expect(true).toBe(true);
  });
});
