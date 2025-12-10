import React, { useState } from 'react';
import CodeBlock from '@theme/CodeBlock';
import styles from './CodeBlock.module.css';

/**
 * Enhanced Code Block Component with Emoji Prefix Support
 *
 * Displays code snippets with:
 * - Syntax highlighting
 * - Copy to clipboard button
 * - Emoji prefix indicators for command types
 * - Line numbers (optional)
 * - Line highlighting (optional)
 *
 * @param {Object} props
 * @param {string} props.children - Code content
 * @param {string} props.language - Programming language for syntax highlighting
 * @param {string} props.title - Optional filename or title
 * @param {boolean} props.showLineNumbers - Show line numbers
 * @param {string} props.highlightLines - Lines to highlight (e.g., "1,3-5")
 * @param {string} props.emojiPrefix - Emoji to show as command type indicator
 * @returns {JSX.Element}
 */
export default function EnhancedCodeBlock({
  children,
  language = 'text',
  title,
  showLineNumbers = false,
  highlightLines,
  emojiPrefix,
  ...props
}) {
  const [copied, setCopied] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(children);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  // Emoji prefix legend
  const emojiLegend = {
    '🔧': 'Setup/Configuration',
    '📦': 'Package Installation',
    '⬇️': 'Download',
    '🚀': 'Launch/Run',
    '✅': 'Verification',
    '🧪': 'Testing',
    '🔍': 'Inspect/Check'
  };

  return (
    <div className={styles.codeBlockContainer}>
      {emojiPrefix && (
        <div className={styles.emojiIndicator} title={emojiLegend[emojiPrefix] || 'Command'}>
          <span className={styles.emoji}>{emojiPrefix}</span>
          <span className={styles.emojiLabel}>{emojiLegend[emojiPrefix]}</span>
        </div>
      )}

      <CodeBlock
        language={language}
        title={title}
        showLineNumbers={showLineNumbers}
        metastring={highlightLines ? `{${highlightLines}}` : undefined}
        {...props}
      >
        {children}
      </CodeBlock>

      <button
        className={`${styles.copyButton} ${copied ? styles.copied : ''}`}
        onClick={handleCopy}
        aria-label="Copy code to clipboard"
      >
        {copied ? '✓ Copied!' : '📋 Copy'}
      </button>
    </div>
  );
}

/**
 * Emoji Prefix Legend Component
 * Displays the meaning of command emoji prefixes
 */
export function EmojiPrefixLegend() {
  return (
    <div className={styles.legend}>
      <h4>Command Type Indicators</h4>
      <ul>
        <li><span>🔧</span> Setup/Configuration</li>
        <li><span>📦</span> Package Installation</li>
        <li><span>⬇️</span> Download</li>
        <li><span>🚀</span> Launch/Run</li>
        <li><span>✅</span> Verification</li>
        <li><span>🧪</span> Testing</li>
        <li><span>🔍</span> Inspect/Check</li>
      </ul>
    </div>
  );
}
