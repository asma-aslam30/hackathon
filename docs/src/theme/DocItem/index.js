import React from 'react';
import DocItem from '@theme-original/DocItem';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

export default function DocItemWrapper(props) {
  return (
    <>
      <div style={{
        display: 'flex',
        justifyContent: 'flex-end',
        marginBottom: '1rem',
        padding: '0 1rem'
      }}>
        <UrduTranslationButton
          contentSelector=".theme-doc-markdown"
          chapterId={props.content?.metadata?.id || 'unknown'}
        />
      </div>
      <DocItem {...props} />
    </>
  );
}
