import React, { useState } from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import styles from './InstallationStep.module.css';

/**
 * Installation Step Component with Platform Switcher
 *
 * Displays installation instructions for multiple platforms (Ubuntu/Windows/macOS)
 * with tab-based switching and collapsible expected output sections
 *
 * @param {Object} props
 * @param {number} props.stepNumber - Step number in sequence
 * @param {string} props.title - Step title
 * @param {string} props.description - Step description
 * @param {Object} props.platforms - Platform-specific instructions
 * @param {string} props.expectedOutput - Expected output after running commands
 * @param {string} props.warning - Optional warning message
 * @param {number} props.estimatedDuration - Estimated time in seconds
 * @returns {JSX.Element}
 */
export default function InstallationStep({
  stepNumber,
  title,
  description,
  platforms = {},
  expectedOutput,
  warning,
  estimatedDuration
}) {
  const [showOutput, setShowOutput] = useState(false);

  const formatDuration = (seconds) => {
    if (seconds < 60) return `~${seconds}s`;
    const minutes = Math.floor(seconds / 60);
    return `~${minutes}min`;
  };

  const availablePlatforms = Object.keys(platforms).filter(p => platforms[p]);

  return (
    <div className={styles.installationStep}>
      <div className={styles.stepHeader}>
        <div className={styles.stepBadge}>Step {stepNumber}</div>
        <div className={styles.stepInfo}>
          <h3 className={styles.stepTitle}>{title}</h3>
          {estimatedDuration && (
            <span className={styles.duration}>
              ⏱️ {formatDuration(estimatedDuration)}
            </span>
          )}
        </div>
      </div>

      {description && (
        <p className={styles.stepDescription}>{description}</p>
      )}

      {warning && (
        <div className={styles.warning}>
          <span className={styles.warningIcon}>⚠️</span>
          <span>{warning}</span>
        </div>
      )}

      {availablePlatforms.length > 0 && (
        <Tabs groupId="operating-system" className={styles.platformTabs}>
          {platforms.ubuntu && (
            <TabItem value="ubuntu" label="🐧 Ubuntu" default>
              <div className={styles.platformContent}>
                {platforms.ubuntu.instructions && (
                  <div className={styles.instructions}>
                    {platforms.ubuntu.instructions}
                  </div>
                )}
                {platforms.ubuntu.commands && platforms.ubuntu.commands.length > 0 && (
                  <div className={styles.commands}>
                    {platforms.ubuntu.commands.map((cmd, index) => (
                      <div key={index} className={styles.commandItem}>
                        <code className={styles.command}>
                          {cmd.emojiPrefix} <strong>{cmd.command}</strong>
                        </code>
                        {cmd.description && (
                          <div className={styles.commandDescription}>
                            {cmd.description}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                )}
              </div>
            </TabItem>
          )}
          {platforms.windows && (
            <TabItem value="windows" label="🪟 Windows">
              <div className={styles.platformContent}>
                {platforms.windows.instructions && (
                  <div className={styles.instructions}>
                    {platforms.windows.instructions}
                  </div>
                )}
                {platforms.windows.commands && platforms.windows.commands.length > 0 && (
                  <div className={styles.commands}>
                    {platforms.windows.commands.map((cmd, index) => (
                      <div key={index} className={styles.commandItem}>
                        <code className={styles.command}>
                          {cmd.emojiPrefix} <strong>{cmd.command}</strong>
                        </code>
                        {cmd.description && (
                          <div className={styles.commandDescription}>
                            {cmd.description}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                )}
              </div>
            </TabItem>
          )}
          {platforms.macos && (
            <TabItem value="macos" label="🍎 macOS">
              <div className={styles.platformContent}>
                {platforms.macos.instructions && (
                  <div className={styles.instructions}>
                    {platforms.macos.instructions}
                  </div>
                )}
                {platforms.macos.commands && platforms.macos.commands.length > 0 && (
                  <div className={styles.commands}>
                    {platforms.macos.commands.map((cmd, index) => (
                      <div key={index} className={styles.commandItem}>
                        <code className={styles.command}>
                          {cmd.emojiPrefix} <strong>{cmd.command}</strong>
                        </code>
                        {cmd.description && (
                          <div className={styles.commandDescription}>
                            {cmd.description}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                )}
              </div>
            </TabItem>
          )}
        </Tabs>
      )}

      {expectedOutput && (
        <div className={styles.outputSection}>
          <button
            className={styles.outputToggle}
            onClick={() => setShowOutput(!showOutput)}
            aria-expanded={showOutput}
          >
            {showOutput ? '▼' : '▶'} Expected Output
          </button>
          {showOutput && (
            <div className={styles.outputContent}>
              <pre><code>{expectedOutput}</code></pre>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
