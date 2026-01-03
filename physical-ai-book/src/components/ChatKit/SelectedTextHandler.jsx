/**
 * SelectedTextHandler component for handling selected text functionality
 */
import React, { useEffect, useState } from 'react';

export const SelectedTextHandler = () => {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      // Only set selected text if it's between 1 and 5000 characters
      if (text && text.length >= 1 && text.length <= 5000) {
        setSelectedText(text);
      } else {
        setSelectedText('');
      }
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  // Function to send selected text to the chat
  const sendSelectedText = () => {
    if (selectedText) {
      // This would typically trigger an event that the ChatInterface can listen to
      // For now, we'll just log it
      console.log('Sending selected text to chat:', selectedText);

      // In a real implementation, this would trigger a custom event or use a state management solution
      const event = new CustomEvent('selectedTextForChat', {
        detail: { text: selectedText }
      });
      document.dispatchEvent(event);

      // Clear the selection
      window.getSelection().removeAllRanges();
      setSelectedText('');
    }
  };

  // Optional: Show a tooltip or button when text is selected
  useEffect(() => {
    if (selectedText) {
      // In a real implementation, you might show a floating button near the selection
      // For now, we'll just log when text is selected
      console.log('Text selected:', selectedText);
    }
  }, [selectedText]);

  return null; // This component doesn't render anything directly
};