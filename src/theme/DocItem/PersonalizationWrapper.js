/**
 * Client-side personalization wrapper
 * Feature: 007-chapter-personalization
 *
 * This component handles all personalization logic and only runs in the browser.
 */

import React, { useState, useCallback, useMemo, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import { useChapterPersonalization } from '@site/src/hooks/useChapterPersonalization';
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

export default function PersonalizationWrapper() {
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

  // Personalization state
  const {
    isPersonalized,
    isLoading,
    error,
    personalizedContent,
    adaptationSummary,
    personalize,
    resetToOriginal
  } = useChapterPersonalization(chapterSlug);

  // Track original content
  const [originalContent, setOriginalContent] = useState(null);

  // Extract content from DOM when personalizing
  const handlePersonalize = useCallback(() => {
    const docContent = document.querySelector('.theme-doc-markdown');
    if (docContent) {
      const rawContent = extractMarkdownFromDOM(docContent);
      setOriginalContent(rawContent);
      personalize(rawContent, docTitle);
    }
  }, [personalize, docTitle]);

  const handleRetry = useCallback(() => {
    if (originalContent) {
      personalize(originalContent, docTitle);
    }
  }, [personalize, originalContent, docTitle]);

  // Hide original content and show personalized when active
  useEffect(() => {
    const docMarkdown = document.querySelector('.theme-doc-markdown');
    if (docMarkdown) {
      if (isPersonalized && personalizedContent) {
        docMarkdown.style.display = 'none';
      } else {
        docMarkdown.style.display = '';
      }
    }
  }, [isPersonalized, personalizedContent]);

  return (
    <>
      {/* Personalization Button */}
      <div className={styles.personalizationWrapper}>
        <PersonalizeButton
          isPersonalized={isPersonalized}
          isLoading={isLoading}
          error={error}
          adaptationSummary={adaptationSummary}
          onPersonalize={handlePersonalize}
          onReset={resetToOriginal}
          onRetry={handleRetry}
        />
      </div>

      {/* Personalized Content (shown when active) */}
      {isPersonalized && personalizedContent && (
        <div className={styles.personalizedContent}>
          <ReactMarkdown
            remarkPlugins={[remarkGfm]}
            components={{
              code: ({ node, inline, className, children, ...props }) => {
                return !inline ? (
                  <pre className={className}>
                    <code className={className} {...props}>
                      {children}
                    </code>
                  </pre>
                ) : (
                  <code className={className} {...props}>
                    {children}
                  </code>
                );
              }
            }}
          >
            {personalizedContent}
          </ReactMarkdown>
        </div>
      )}
    </>
  );
}
