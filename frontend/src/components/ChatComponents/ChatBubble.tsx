import React from 'react';
import styles from './ChatComponents.module.css';

interface ChatBubbleProps {
  role: 'user' | 'assistant';
  content: string;
  timestamp?: string;
  sources?: Array<{
    title: string;
    source_url?: string;
  }>;
}

const ChatBubble: React.FC<ChatBubbleProps> = ({
  role,
  content,
  timestamp,
  sources,
}) => {
  return (
    <div
      className={`${styles.bubble} ${
        role === 'user' ? styles.userBubble : styles.assistantBubble
      }`}
    >
      <div className={styles.bubbleContent}>{content}</div>
      {sources && sources.length > 0 && (
        <div className={styles.bubbleSources}>
          {sources.slice(0, 3).map((source, i) => (
            <span key={i} className={styles.sourceChip}>
              {source.title}
            </span>
          ))}
        </div>
      )}
      {timestamp && (
        <div className={styles.timestamp}>
          {new Date(timestamp).toLocaleTimeString()}
        </div>
      )}
    </div>
  );
};

export default ChatBubble;
