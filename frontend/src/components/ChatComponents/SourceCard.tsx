import React from 'react';
import styles from './ChatComponents.module.css';

interface SourceCardProps {
  title: string;
  content_preview?: string;
  source_url?: string;
  relevance_score?: number;
}

const SourceCard: React.FC<SourceCardProps> = ({
  title,
  content_preview,
  source_url,
  relevance_score,
}) => {
  return (
    <div className={styles.sourceCard}>
      <div className={styles.sourceTitle}>
        {source_url ? (
          <a href={source_url} target="_blank" rel="noopener noreferrer">
            {title}
          </a>
        ) : (
          title
        )}
      </div>
      {content_preview && (
        <div className={styles.sourcePreview}>{content_preview}</div>
      )}
      {relevance_score !== undefined && (
        <div className={styles.sourceScore}>
          Relevance: {Math.round(relevance_score * 100)}%
        </div>
      )}
    </div>
  );
};

export default SourceCard;
