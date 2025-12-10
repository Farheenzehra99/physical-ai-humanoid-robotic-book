import React from 'react';
import styles from './HardwareTable.module.css';

/**
 * Accessible Hardware Requirements Table Component
 *
 * Displays hardware specifications across tiers (minimum/recommended/extreme)
 * Complies with WCAG 2.1 AA accessibility standards
 *
 * @param {Object} props
 * @param {string} props.title - Table title
 * @param {string} props.caption - Accessible caption describing table content
 * @param {Array} props.requirements - Array of hardware requirement objects
 * @returns {JSX.Element}
 */
export default function HardwareTable({ title, caption, requirements = [] }) {
  // Group requirements by tier for easy rendering
  const tiers = ['minimum', 'recommended', 'extreme'];
  const componentTypes = ['gpu', 'cpu', 'ram', 'storage', 'os'];

  const getRequirement = (componentType, tier) => {
    return requirements.find(
      r => r.componentType === componentType && r.tier === tier
    );
  };

  return (
    <div className={styles.tableContainer}>
      {title && <h3 className={styles.tableTitle}>{title}</h3>}

      <table className={styles.hardwareTable}>
        <caption className={styles.tableCaption}>
          {caption || 'Hardware requirements comparison table'}
        </caption>

        <thead>
          <tr>
            <th scope="col">Component</th>
            <th scope="col">Minimum</th>
            <th scope="col">Recommended</th>
            <th scope="col">Extreme Performance</th>
          </tr>
        </thead>

        <tbody>
          {componentTypes.map(componentType => (
            <tr key={componentType}>
              <th scope="row" className={styles.componentLabel}>
                {componentType.toUpperCase()}
              </th>
              {tiers.map(tier => {
                const req = getRequirement(componentType, tier);
                return (
                  <td key={tier} className={styles[`tier-${tier}`]}>
                    {req ? (
                      <div className={styles.specCell}>
                        <div className={styles.specText}>{req.specification}</div>
                        {req.estimatedPrice && (
                          <div className={styles.priceTag}>
                            ${req.estimatedPrice} USD
                          </div>
                        )}
                        {req.notes && (
                          <div className={styles.notes}>{req.notes}</div>
                        )}
                      </div>
                    ) : (
                      <span className={styles.notApplicable}>N/A</span>
                    )}
                  </td>
                );
              })}
            </tr>
          ))}
        </tbody>
      </table>

      <div className={styles.tableLegend} aria-label="Table legend">
        <p><strong>Note</strong>: Prices are estimates as of December 2025 and may vary by retailer.</p>
      </div>
    </div>
  );
}
