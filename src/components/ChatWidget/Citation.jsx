/**
 * Citation component - Displays source citation with link
 * Feature: 005-frontend-chat
 */

import React from 'react';
import styles from './styles.module.css';

/**
 * Renders a citation link
 * @param {Object} props
 * @param {Object} props.citation - Citation object with title and source_url
 */
export default function Citation({ citation }) {
  const { title, source_url } = citation;

  // Determine if link is internal (same domain) or external
  const isInternal = source_url && (
    source_url.startsWith('/') ||
    source_url.startsWith(window.location.origin)
  );

  // Internal links open in same tab, external in new tab
  const linkProps = isInternal
    ? {}
    : { target: '_blank', rel: 'noopener noreferrer' };

  return (
    <a
      href={source_url}
      className={styles.citation}
      {...linkProps}
    >
      {title || source_url}
    </a>
  );
}
