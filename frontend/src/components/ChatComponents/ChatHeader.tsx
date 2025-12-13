import React from 'react';
import styles from './ChatComponents.module.css';

interface ChatHeaderProps {
  title?: string;
  subtitle?: string;
  onClose?: () => void;
}

const ChatHeader: React.FC<ChatHeaderProps> = ({
  title = 'AI Assistant',
  subtitle,
  onClose,
}) => {
  return (
    <div className={styles.header}>
      <div className={styles.headerIcon}>
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <circle cx="12" cy="12" r="10"></circle>
          <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"></path>
          <line x1="12" y1="17" x2="12.01" y2="17"></line>
        </svg>
      </div>
      <div className={styles.headerText}>
        <span className={styles.headerTitle}>{title}</span>
        {subtitle && <span className={styles.headerSubtitle}>{subtitle}</span>}
      </div>
      {onClose && (
        <button className={styles.closeBtn} onClick={onClose} aria-label="Close chat">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        </button>
      )}
    </div>
  );
};

export default ChatHeader;
