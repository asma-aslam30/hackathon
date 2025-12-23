import React from 'react';

const BackgroundFields = ({ formData, errors, handleChange, handleNestedChange }) => {
  // Common options for various fields
  const experienceLevels = [
    { value: 'beginner', label: 'Beginner' },
    { value: 'intermediate', label: 'Intermediate' },
    { value: 'advanced', label: 'Advanced' },
    { value: 'expert', label: 'Expert' }
  ];

  const primaryDomains = [
    { value: 'web', label: 'Web Development' },
    { value: 'mobile', label: 'Mobile Development' },
    { value: 'data', label: 'Data Science' },
    { value: 'ai', label: 'AI/ML' },
    { value: 'devops', label: 'DevOps' },
    { value: 'security', label: 'Security' },
    { value: 'iot', label: 'IoT' },
    { value: 'embedded', label: 'Embedded Systems' }
  ];

  const osOptions = [
    { value: 'windows', label: 'Windows' },
    { value: 'macos', label: 'macOS' },
    { value: 'linux', label: 'Linux' },
    { value: 'unix', label: 'Unix' }
  ];

  const workStyles = [
    { value: 'remote', label: 'Remote' },
    { value: 'hybrid', label: 'Hybrid' },
    { value: 'onsite', label: 'On-site' }
  ];

  const commonSoftwareTools = [
    'VSCode', 'IntelliJ', 'PyCharm', 'Eclipse', 'Xcode', 'Visual Studio',
    'Sublime Text', 'Vim', 'Emacs', 'Atom', 'Notepad++', 'WebStorm',
    'Android Studio', 'Xamarin', 'Unity', 'Unreal Engine', 'Blender',
    'Figma', 'Sketch', 'Adobe XD', 'Git', 'Docker', 'Kubernetes'
  ];

  const commonProgrammingLanguages = [
    'Python', 'JavaScript', 'TypeScript', 'Java', 'C#', 'C++', 'C',
    'Go', 'Rust', 'Swift', 'Kotlin', 'Dart', 'Ruby', 'PHP', 'R',
    'MATLAB', 'SQL', 'NoSQL', 'Shell', 'PowerShell'
  ];

  return (
    <div className="background-fields">
      {/* Programming Experience */}
      <div className="form-group">
        <label>Programming Experience Level</label>
        <div className="radio-group">
          {experienceLevels.map(level => (
            <label key={level.value} className="radio-option">
              <input
                type="radio"
                name="experience_level"
                value={level.value}
                checked={formData.experience_level === level.value}
                onChange={(e) => handleChange('experience_level', e.target.value)}
              />
              {level.label}
            </label>
          ))}
        </div>
        {errors.experience_level && <div className="error-message">{errors.experience_level}</div>}
      </div>

      {/* Primary Domain */}
      <div className="form-group">
        <label htmlFor="primary_domain">Primary Technical Domain</label>
        <select
          id="primary_domain"
          value={formData.primary_domain}
          onChange={(e) => handleChange('primary_domain', e.target.value)}
          className={errors.primary_domain ? 'error' : ''}
        >
          <option value="">Select your primary domain</option>
          {primaryDomains.map(domain => (
            <option key={domain.value} value={domain.value}>
              {domain.label}
            </option>
          ))}
        </select>
        {errors.primary_domain && <div className="error-message">{errors.primary_domain}</div>}
      </div>

      {/* Software Tools */}
      <div className="form-group">
        <label>Software Tools You Use</label>
        <div className="checkbox-group">
          {commonSoftwareTools.map(tool => (
            <label key={tool} className="checkbox-option">
              <input
                type="checkbox"
                value={tool}
                checked={formData.software_tools.includes(tool)}
                onChange={(e) => {
                  const updatedTools = e.target.checked
                    ? [...formData.software_tools, tool]
                    : formData.software_tools.filter(t => t !== tool);
                  handleChange('software_tools', updatedTools);
                }}
              />
              {tool}
            </label>
          ))}
        </div>
        {errors.software_tools && <div className="error-message">{errors.software_tools}</div>}
      </div>

      {/* Programming Languages */}
      <div className="form-group">
        <label>Programming Languages You Know</label>
        <div className="checkbox-group">
          {commonProgrammingLanguages.map(lang => (
            <label key={lang} className="checkbox-option">
              <input
                type="checkbox"
                value={lang}
                checked={formData.programming_languages.includes(lang)}
                onChange={(e) => {
                  const updatedLanguages = e.target.checked
                    ? [...formData.programming_languages, lang]
                    : formData.programming_languages.filter(l => l !== lang);
                  handleChange('programming_languages', updatedLanguages);
                }}
              />
              {lang}
            </label>
          ))}
        </div>
        {errors.programming_languages && <div className="error-message">{errors.programming_languages}</div>}
      </div>

      {/* Hardware Setup */}
      <div className="form-group">
        <label>Hardware Setup</label>
        <div className="nested-fields">
          <div className="form-row">
            <div className="form-group">
              <label htmlFor="os">Operating System</label>
              <select
                id="os"
                value={formData.hardware_setup.os}
                onChange={(e) => handleNestedChange('hardware_setup', 'os', e.target.value)}
              >
                <option value="">Select OS</option>
                {osOptions.map(os => (
                  <option key={os.value} value={os.value}>{os.label}</option>
                ))}
              </select>
            </div>
            <div className="form-group">
              <label htmlFor="cpu">CPU</label>
              <input
                type="text"
                id="cpu"
                value={formData.hardware_setup.cpu}
                onChange={(e) => handleNestedChange('hardware_setup', 'cpu', e.target.value)}
                placeholder="e.g., Intel i7-10700K"
              />
            </div>
          </div>
          <div className="form-row">
            <div className="form-group">
              <label htmlFor="ram">RAM</label>
              <input
                type="text"
                id="ram"
                value={formData.hardware_setup.ram}
                onChange={(e) => handleNestedChange('hardware_setup', 'ram', e.target.value)}
                placeholder="e.g., 16GB DDR4"
              />
            </div>
            <div className="form-group">
              <label htmlFor="gpu">GPU</label>
              <input
                type="text"
                id="gpu"
                value={formData.hardware_setup.gpu}
                onChange={(e) => handleNestedChange('hardware_setup', 'gpu', e.target.value)}
                placeholder="e.g., NVIDIA RTX 3080"
              />
            </div>
          </div>
        </div>
      </div>

      {/* Technical Preferences */}
      <div className="form-group">
        <label>Technical Preferences</label>
        <div className="nested-fields">
          <div className="form-row">
            <div className="form-group">
              <label htmlFor="ide_preference">Preferred IDE/Editor</label>
              <input
                type="text"
                id="ide_preference"
                value={formData.technical_preferences.ide_preference}
                onChange={(e) => handleNestedChange('technical_preferences', 'ide_preference', e.target.value)}
                placeholder="e.g., VSCode, IntelliJ, etc."
              />
            </div>
            <div className="form-group">
              <label htmlFor="work_style">Work Style</label>
              <select
                id="work_style"
                value={formData.technical_preferences.work_style}
                onChange={(e) => handleNestedChange('technical_preferences', 'work_style', e.target.value)}
              >
                <option value="">Select work style</option>
                {workStyles.map(style => (
                  <option key={style.value} value={style.value}>{style.label}</option>
                ))}
              </select>
            </div>
          </div>
          <div className="form-group">
            <label htmlFor="preferred_environment">Preferred Development Environment</label>
            <input
              type="text"
              id="preferred_environment"
              value={formData.technical_preferences.preferred_environment}
              onChange={(e) => handleNestedChange('technical_preferences', 'preferred_environment', e.target.value)}
              placeholder="e.g., Linux terminal, Windows GUI, etc."
            />
          </div>
        </div>
      </div>
    </div>
  );
};

export default BackgroundFields;