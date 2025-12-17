import React from 'react';
import clsx from 'clsx';
import styles from './SummaryBox.module.css';

const SummaryBox = ({ title, children }) => {
  return (
    <div className={clsx('margin--vertical', styles.summaryBox)}>
      <div className={styles.header}>
        <h3>{title}</h3>
      </div>
      <div className={styles.content}>{children}</div>
    </div>
  );
};

export default SummaryBox;
