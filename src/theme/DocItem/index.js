/**
 * Swizzled DocItem component with personalization and translation support
 * Features: 007-chapter-personalization, 008-urdu-translation
 */

import React from 'react';
import DocItem from '@theme-original/DocItem';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Lazy load components to avoid SSR issues
const PersonalizationWrapper = React.lazy(() => import('./PersonalizationWrapper'));
const TranslationWrapper = React.lazy(() => import('./TranslationWrapper'));

export default function DocItemWrapper(props) {
  return (
    <>
      {/* Action Buttons Row - Both buttons side by side */}
      <BrowserOnly fallback={null}>
        {() => (
          <div style={{
            display: 'flex',
            flexDirection: 'row',
            gap: '1rem',
            flexWrap: 'wrap',
            maxWidth: 'var(--ifm-container-width)',
            margin: '0 auto',
            padding: '0 var(--ifm-spacing-horizontal)',
            marginBottom: '1rem'
          }}>
            <React.Suspense fallback={null}>
              <TranslationWrapper />
            </React.Suspense>
            <React.Suspense fallback={null}>
              <PersonalizationWrapper />
            </React.Suspense>
          </div>
        )}
      </BrowserOnly>

      {/* Always render the original DocItem */}
      <DocItem {...props} />
    </>
  );
}
