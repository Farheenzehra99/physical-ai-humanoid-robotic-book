/**
 * Client-side translation wrapper
 * Feature: 008-urdu-translation
 *
 * This component handles all translation logic and only runs in the browser.
 */

import React, { useState, useCallback, useMemo, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import TranslateButton from '@site/src/components/TranslateButton/TranslateButton';
import { useChapterTranslation } from '@site/src/hooks/useChapterTranslation';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import styles from './styles.module.css';

/**
 * Extract markdown-like content from DOM
 * This is a simplified extraction - production should use source files
 */
function extractMarkdownFromDOM(element) {
  // Clone to avoid modifying the actual DOM
  const clone = element.cloneNode(true);

  // Remove elements we don't want
  clone.querySelectorAll('script, style, .hash-link').forEach(el => el.remove());

  // Convert to text with basic markdown structure
  let markdown = '';

  clone.querySelectorAll('h1, h2, h3, h4, h5, h6, p, pre, ul, ol, blockquote').forEach(el => {
    const tag = el.tagName.toLowerCase();

    if (tag.startsWith('h')) {
      const level = parseInt(tag[1]);
      markdown += '#'.repeat(level) + ' ' + el.textContent.trim() + '\n\n';
    } else if (tag === 'p') {
      markdown += el.textContent.trim() + '\n\n';
    } else if (tag === 'pre') {
      const code = el.querySelector('code');
      const lang = code?.className?.match(/language-(\w+)/)?.[1] || '';
      markdown += '```' + lang + '\n' + (code?.textContent || el.textContent) + '\n```\n\n';
    } else if (tag === 'ul' || tag === 'ol') {
      el.querySelectorAll('li').forEach((li, i) => {
        const prefix = tag === 'ol' ? `${i + 1}. ` : '- ';
        markdown += prefix + li.textContent.trim() + '\n';
      });
      markdown += '\n';
    } else if (tag === 'blockquote') {
      markdown += '> ' + el.textContent.trim().split('\n').join('\n> ') + '\n\n';
    }
  });

  return markdown.trim();
}

export default function TranslationWrapper() {
  const location = useLocation();
  const [docTitle, setDocTitle] = useState('');

  // Extract chapter slug from pathname
  const chapterSlug = useMemo(() => {
    return location.pathname.replace(/^\/docs\//, '').replace(/\/$/, '');
  }, [location.pathname]);

  // Get title from document after mount
  useEffect(() => {
    const titleElement = document.querySelector('.theme-doc-markdown h1');
    if (titleElement) {
      setDocTitle(titleElement.textContent || '');
    }
  }, [location.pathname]);

  // Translation state
  const {
    isTranslated,
    isLoading,
    error,
    translatedContent,
    translationSummary,
    translate,
    resetToOriginal,
    toggleTranslation
  } = useChapterTranslation(chapterSlug);

  // Track original content
  const [originalContent, setOriginalContent] = useState(null);

  // Handle toggle between original and translated
  const handleToggle = useCallback(() => {
    if (isTranslated) {
      // Switch back to original
      resetToOriginal();
    } else {
      // Translate
      const docContent = document.querySelector('.theme-doc-markdown');
      if (docContent) {
        const rawContent = extractMarkdownFromDOM(docContent);
        setOriginalContent(rawContent);
        translate(rawContent, docTitle);
      }
    }
  }, [isTranslated, resetToOriginal, translate, docTitle]);

  const handleRetry = useCallback(() => {
    if (originalContent) {
      translate(originalContent, docTitle);
    }
  }, [translate, originalContent, docTitle]);

  // Hide original content and show translated when active
  useEffect(() => {
    const docMarkdown = document.querySelector('.theme-doc-markdown');
    if (docMarkdown) {
      if (isTranslated && translatedContent) {
        docMarkdown.style.display = 'none';
      } else {
        docMarkdown.style.display = '';
      }
    }
  }, [isTranslated, translatedContent]);

  return (
    <>
      {/* Translation Button */}
      <div className={styles.translationWrapper}>
        <TranslateButton
          isTranslated={isTranslated}
          isLoading={isLoading}
          error={error}
          translationSummary={translationSummary}
          onTranslate={handleToggle}
          onReset={resetToOriginal}
          onRetry={handleRetry}
          onToggle={handleToggle}
        />
      </div>

      {/* Translated Content (shown when active) */}
      {isTranslated && translatedContent && (
        <div className={styles.translatedContent} dir="rtl" lang="ur">
          <ReactMarkdown
            remarkPlugins={[remarkGfm]}
            components={{
              code: ({ node, inline, className, children, ...props }) => {
                return !inline ? (
                  <pre className={className} dir="ltr">
                    <code className={className} {...props}>
                      {children}
                    </code>
                  </pre>
                ) : (
                  <code className={className} dir="ltr" {...props}>
                    {children}
                  </code>
                );
              }
            }}
          >
            {translatedContent}
          </ReactMarkdown>
        </div>
      )}
    </>
  );
}
