import React from 'react';
import ChapterContent from '../src/components/ChapterContent/ChapterContent';

/**
 * Example Docusaurus component using the personalization feature
 * This would typically be integrated into a Docusaurus MDX file
 */
const PersonalizedChapter = ({ chapterId, defaultContent }) => {
  return (
    <div className="container">
      <ChapterContent
        chapterId={chapterId}
        defaultContent={defaultContent}
      />
    </div>
  );
};

export default PersonalizedChapter;