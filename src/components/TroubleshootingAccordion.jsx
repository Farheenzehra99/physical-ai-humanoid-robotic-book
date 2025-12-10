import React, { useState } from 'react';
import styles from './TroubleshootingAccordion.module.css';

/**
 * Troubleshooting Accordion Component
 *
 * Displays common problems and solutions in an expandable accordion format
 * Supports multiple solutions per problem with step-by-step instructions
 *
 * @param {Object} props
 * @param {Array} props.entries - Array of troubleshooting entry objects
 * @returns {JSX.Element}
 */
export default function TroubleshootingAccordion({ entries = [] }) {
  const [expandedId, setExpandedId] = useState(null);

  const toggleEntry = (id) => {
    setExpandedId(expandedId === id ? null : id);
  };

  const getSeverityColor = (severity) => {
    switch (severity) {
      case 'blocker': return '#FF4444';
      case 'major': return '#FF9900';
      case 'minor': return '#FFD700';
      default: return 'var(--ifm-color-primary)';
    }
  };

  const getFrequencyLabel = (frequency) => {
    switch (frequency) {
      case 'common': return '🔥 Common';
      case 'occasional': return '⚡ Occasional';
      case 'rare': return '🌙 Rare';
      default: return '';
    }
  };

  return (
    <div className={styles.troubleshootingContainer}>
      {entries.map((entry, index) => {
        const isExpanded = expandedId === entry.id;

        return (
          <div
            key={entry.id || index}
            className={`${styles.accordionItem} ${isExpanded ? styles.expanded : ''}`}
          >
            <button
              className={styles.accordionHeader}
              onClick={() => toggleEntry(entry.id || index)}
              aria-expanded={isExpanded}
              style={{ borderLeftColor: getSeverityColor(entry.severity) }}
            >
              <div className={styles.headerContent}>
                <div className={styles.problemTitle}>
                  <span className={styles.expandIcon}>{isExpanded ? '▼' : '▶'}</span>
                  <span className={styles.problemText}>{entry.problem}</span>
                </div>
                <div className={styles.metadata}>
                  {entry.frequency && (
                    <span className={styles.frequencyBadge}>
                      {getFrequencyLabel(entry.frequency)}
                    </span>
                  )}
                  {entry.severity && (
                    <span
                      className={styles.severityBadge}
                      style={{ backgroundColor: getSeverityColor(entry.severity) }}
                    >
                      {entry.severity}
                    </span>
                  )}
                </div>
              </div>
            </button>

            {isExpanded && (
              <div className={styles.accordionContent}>
                {/* Cause */}
                {entry.cause && (
                  <div className={styles.section}>
                    <h4 className={styles.sectionTitle}>Cause</h4>
                    <p className={styles.causeText}>{entry.cause}</p>
                  </div>
                )}

                {/* Symptoms */}
                {entry.symptoms && entry.symptoms.length > 0 && (
                  <div className={styles.section}>
                    <h4 className={styles.sectionTitle}>Symptoms</h4>
                    <ul className={styles.symptomsList}>
                      {entry.symptoms.map((symptom, idx) => (
                        <li key={idx}>{symptom}</li>
                      ))}
                    </ul>
                  </div>
                )}

                {/* Solutions */}
                {entry.solutions && entry.solutions.length > 0 && (
                  <div className={styles.section}>
                    <h4 className={styles.sectionTitle}>Solutions</h4>
                    {entry.solutions.map((solution, idx) => (
                      <div
                        key={idx}
                        className={`${styles.solution} ${solution.isDefinitive ? styles.definitiveSolution : ''}`}
                      >
                        <div className={styles.solutionHeader}>
                          <span className={styles.solutionNumber}>Solution {idx + 1}</span>
                          {solution.isDefinitive && (
                            <span className={styles.definitiveLabel}>✓ Recommended</span>
                          )}
                        </div>
                        <p className={styles.solutionDescription}>{solution.description}</p>
                        {solution.commands && solution.commands.length > 0 && (
                          <div className={styles.commandsSection}>
                            {solution.commands.map((cmd, cmdIdx) => (
                              <div key={cmdIdx} className={styles.command}>
                                {cmd.emojiPrefix && (
                                  <span className={styles.commandEmoji}>{cmd.emojiPrefix}</span>
                                )}
                                <code className={styles.commandCode}>{cmd.command}</code>
                              </div>
                            ))}
                          </div>
                        )}
                        {solution.expectedResult && (
                          <div className={styles.expectedResult}>
                            <strong>Expected result:</strong> {solution.expectedResult}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                )}

                {/* Related Errors */}
                {entry.relatedErrors && entry.relatedErrors.length > 0 && (
                  <details className={styles.relatedErrors}>
                    <summary>Related Error Messages</summary>
                    <ul>
                      {entry.relatedErrors.map((error, idx) => (
                        <li key={idx}><code>{error}</code></li>
                      ))}
                    </ul>
                  </details>
                )}
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
}
