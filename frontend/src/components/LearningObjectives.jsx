import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjectives.module.css';

const LearningObjectives = ({ children }) => {
  return (
    <div className={clsx('margin--vertical', styles.learningObjectives)}>
      <div className={styles.header}>
        <h3>Learning Objectives</h3>
      </div>
      <div className={styles.content}>{children}</div>
    </div>
  );
};

export default LearningObjectives;
