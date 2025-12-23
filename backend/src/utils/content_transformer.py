"""
Content transformation utilities for personalizing chapter content based on user background
"""

import re
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from enum import Enum


class ExperienceLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
    EXPERT = "expert"


@dataclass
class UserProfile:
    user_id: str
    programming_languages: List[str]
    experience_level: str
    domain_expertise: List[str]
    tools_familiarity: List[str]
    os_familiarity: List[str]


class ContentTransformer:
    """
    Transforms content based on user's background profile
    """

    def __init__(self):
        # Define mappings for different programming languages
        self.language_examples = {
            "python": {
                "web_dev": [
                    {"original": r"function (\w+)\(\)", "replacement": r"def $1():"},
                    {"original": r"console\.log\((.*?)\)", "replacement": r"print($1)"},
                    {"original": r"let (\w+) = (.*?);", "replacement": r"$1 = $2"},
                    {"original": r"const (\w+) = (.*?);", "replacement": r"$1 = $2"},
                ],
                "data_science": [
                    {"original": r"function (\w+)\(\)", "replacement": r"def $1():"},
                    {"original": r"console\.log\((.*?)\)", "replacement": r"print($1)"},
                    {"original": r"JSON\.parse\((.*?)\)", "replacement": r"json.loads($1)"},
                ]
            },
            "javascript": {
                "web_dev": [
                    {"original": r"def (\w+)\(\):", "replacement": r"function $1() {"},
                    {"original": r"print\((.*?)\)", "replacement": r"console.log($1)"},
                    {"original": r"(\w+) = (.*?)$", "replacement": r"let $1 = $2;"},
                ]
            },
            "java": {
                "enterprise": [
                    {"original": r"function (\w+)\(\)", "replacement": r"public void $1() {"},
                    {"original": r"def (\w+)\(\):", "replacement": r"public void $1() {"},
                    {"original": r"console\.log\((.*?)\)", "replacement": r"System.out.println($1)"},
                ]
            }
        }

        # Define complexity mappings based on experience level
        self.complexity_mappings = {
            ExperienceLevel.BEGINNER.value: {
                "replace_complex": [
                    {"original": r"utilize", "replacement": r"use"},
                    {"original": r"implement", "replacement": r"create"},
                    {"original": r"leverage", "replacement": r"use"},
                    {"original": r"function", "replacement": r"function/method"},
                ],
                "add_explanations": [
                    {"pattern": r"(\w+)\(\)", "explanation": r"$1() - This is a function call"},
                ]
            },
            ExperienceLevel.INTERMEDIATE.value: {
                "replace_complex": [],
                "add_explanations": []
            },
            ExperienceLevel.ADVANCED.value: {
                "replace_complex": [
                    {"original": r"use", "replacement": r"utilize"},
                    {"original": r"create", "replacement": r"implement"},
                ],
                "add_explanations": []
            },
            ExperienceLevel.EXPERT.value: {
                "replace_complex": [
                    {"original": r"use", "replacement": r"leverage"},
                    {"original": r"create", "replacement": r"implement"},
                ],
                "add_explanations": []
            }
        }

    def personalize_content(self, content: str, user_profile: UserProfile) -> Dict[str, Any]:
        """
        Personalize content based on user profile
        """
        # Determine the most relevant domain for the user
        primary_domain = self._get_primary_domain(user_profile.domain_expertise)

        # Transform content based on programming language preference
        transformed_content = self._transform_by_language(content, user_profile, primary_domain)

        # Adjust complexity based on experience level
        transformed_content = self._adjust_complexity(transformed_content, user_profile.experience_level)

        # Apply domain-specific customizations
        transformed_content = self._apply_domain_customizations(transformed_content, user_profile, primary_domain)

        return {
            "personalized_content": transformed_content,
            "personalization_metadata": {
                "rules_applied": ["language_substitution", "complexity_adjustment", "domain_customization"],
                "profile_used": {
                    "experience_level": user_profile.experience_level,
                    "programming_languages": user_profile.programming_languages,
                    "domain_expertise": user_profile.domain_expertise
                }
            }
        }

    def _get_primary_domain(self, domains: List[str]) -> str:
        """
        Determine the primary domain from user's expertise
        """
        if not domains:
            return "general"

        # Map domain expertise to standardized categories
        domain_mapping = {
            "web development": "web_dev",
            "web_dev": "web_dev",
            "data science": "data_science",
            "data_science": "data_science",
            "machine learning": "data_science",
            "ml": "data_science",
            "ai": "data_science",
            "artificial intelligence": "data_science",
            "enterprise": "enterprise",
            "backend": "web_dev",
            "frontend": "web_dev",
            "fullstack": "web_dev",
        }

        for domain in domains:
            if domain.lower() in domain_mapping:
                return domain_mapping[domain.lower()]

        return "general"

    def _transform_by_language(self, content: str, user_profile: UserProfile, domain: str) -> str:
        """
        Transform content based on user's preferred programming language
        """
        transformed_content = content

        for language in user_profile.programming_languages:
            language_key = language.lower()
            if language_key in self.language_examples:
                domain_examples = self.language_examples[language_key].get(domain, [])

                for example in domain_examples:
                    transformed_content = re.sub(
                        example["original"],
                        example["replacement"],
                        transformed_content
                    )

        return transformed_content

    def _adjust_complexity(self, content: str, experience_level: str) -> str:
        """
        Adjust content complexity based on user's experience level
        """
        if experience_level not in self.complexity_mappings:
            return content

        transformations = self.complexity_mappings[experience_level]
        transformed_content = content

        # Apply word replacements
        for replacement in transformations.get("replace_complex", []):
            transformed_content = re.sub(
                replacement["original"],
                replacement["replacement"],
                transformed_content,
                flags=re.IGNORECASE
            )

        # Add explanations for beginners
        if experience_level == ExperienceLevel.BEGINNER.value:
            for explanation in transformations.get("add_explanations", []):
                transformed_content = re.sub(
                    explanation["pattern"],
                    f"{explanation['explanation']}\n\n$0",
                    transformed_content
                )

        return transformed_content

    def _apply_domain_customizations(self, content: str, user_profile: UserProfile, domain: str) -> str:
        """
        Apply domain-specific customizations to the content
        """
        # Add domain-specific examples or context
        if domain == "web_dev":
            # Add web development specific context
            content = content.replace(
                "code example",
                f"web development code example (for {', '.join(user_profile.programming_languages)})"
            )
        elif domain == "data_science":
            # Add data science specific context
            content = content.replace(
                "code example",
                f"data science code example (for {', '.join(user_profile.programming_languages)})"
            )

        return content


# Global instance for use in services
content_transformer = ContentTransformer()