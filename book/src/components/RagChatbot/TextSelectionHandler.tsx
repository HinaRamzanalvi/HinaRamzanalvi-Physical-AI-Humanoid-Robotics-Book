import React, { useEffect } from 'react';

interface TextSelectionHandlerProps {
  onTextSelected: (selectedText: string) => void;
}

const TextSelectionHandler: React.FC<TextSelectionHandlerProps> = ({ onTextSelected }) => {
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim() || '';
      onTextSelected(selectedText);
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, [onTextSelected]);

  return null; // This component doesn't render anything visible
};

export default TextSelectionHandler;