import React from 'react';
import clsx from 'clsx';
import styles from './CaseStudy.module.css';

const CaseStudy = ({ title, children }) => {
  return (
    <div className={clsx('margin--vertical', styles.caseStudy)}>
      <div className={styles.header}>
        <h3>{title}</h3>
      </div>
      <div className={styles.content}>{children}</div>
    </div>
  );
};

export default CaseStudy;
