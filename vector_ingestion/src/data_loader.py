import os
from typing import List, Dict, Any
import json


class DataLoader:
    """
    Load textbook content from various sources (files, databases, etc.)
    """

    def load_from_text_file(self, file_path: str) -> List[Dict[str, Any]]:
        """
        Load textbook content from a text file
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            # For now, we'll return a single chunk
            # In a real implementation, this would parse the textbook structure
            return [{
                "chapter_id": "sample-chapter",
                "section_title": "Sample Section",
                "content_text": content,
                "page_reference": f"file://{file_path}"
            }]
        except Exception as e:
            print(f"Error loading from text file: {e}")
            return []

    def load_from_json_file(self, file_path: str) -> List[Dict[str, Any]]:
        """
        Load textbook content from a JSON file with structured format
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                data = json.load(file)

            # Expecting format: [{"chapter_id": "...", "section_title": "...", "content_text": "...", "page_reference": "..."}]
            if isinstance(data, list):
                return data
            elif isinstance(data, dict) and "chapters" in data:
                # If it's a book structure with chapters
                result = []
                for chapter in data["chapters"]:
                    for section in chapter.get("sections", []):
                        result.append({
                            "chapter_id": chapter["id"],
                            "section_title": section["title"],
                            "content_text": section["content"],
                            "page_reference": section.get("page_reference", f"Chapter {chapter['id']}")
                        })
                return result
            else:
                raise ValueError("JSON format not recognized")
        except Exception as e:
            print(f"Error loading from JSON file: {e}")
            return []

    def load_from_directory(self, directory_path: str) -> List[Dict[str, Any]]:
        """
        Load textbook content from all text files in a directory
        """
        all_content = []

        for root, dirs, files in os.walk(directory_path):
            for file in files:
                if file.endswith(('.txt', '.md')):
                    file_path = os.path.join(root, file)
                    if file.endswith('.txt'):
                        content = self.load_from_text_file(file_path)
                    elif file.endswith('.md'):
                        # For now, treat markdown as text
                        content = self.load_from_text_file(file_path)
                    else:
                        continue
                    all_content.extend(content)

        return all_content

    def load_sample_content(self) -> List[Dict[str, Any]]:
        """
        Load sample textbook content for testing purposes
        """
        sample_content = [
            {
                "chapter_id": "ch1-introduction",
                "section_title": "Introduction to Physical AI",
                "content_text": """
                Physical AI is an interdisciplinary field that combines artificial intelligence with physical systems.
                It encompasses robotics, embodied intelligence, and the interaction between AI systems and the physical world.
                Unlike traditional AI that operates primarily in digital environments, Physical AI must deal with the complexities
                of the real world including uncertainty, dynamics, and physical constraints.

                Key aspects of Physical AI include:
                - Embodied cognition: How physical form influences intelligence
                - Sensorimotor learning: Learning through interaction with the environment
                - Real-time decision making: Making decisions under temporal constraints
                - Robustness: Operating reliably in unstructured environments
                """,
                "page_reference": "Chapter 1, Page 1-15"
            },
            {
                "chapter_id": "ch2-humanoid-robots",
                "section_title": "Humanoid Robot Design Principles",
                "content_text": """
                Humanoid robots are robots designed to resemble and mimic human behavior and appearance.
                They typically have a head, torso, two arms, and two legs, though some variations exist.
                The design of humanoid robots involves multiple engineering disciplines including mechanical engineering,
                electrical engineering, computer science, and cognitive science.

                Design considerations include:
                - Kinematics: The study of motion without considering forces
                - Dynamics: How forces affect motion
                - Balance and stability: Maintaining posture and gait
                - Actuation: How joints are powered and controlled
                - Control systems: Algorithms for movement and behavior
                """,
                "page_reference": "Chapter 2, Page 16-35"
            },
            {
                "chapter_id": "ch3-ai-fundamentals",
                "section_title": "AI Techniques for Robotics",
                "content_text": """
                Artificial intelligence techniques play a crucial role in enabling autonomous behavior in robots.
                These techniques allow robots to perceive their environment, make decisions, plan actions,
                and learn from experience.

                Key AI techniques for robotics include:
                - Perception: Computer vision, sensor fusion, object recognition
                - Planning: Path planning, motion planning, task planning
                - Control: Feedback control, adaptive control, learning-based control
                - Learning: Reinforcement learning, imitation learning, transfer learning
                - Reasoning: Knowledge representation, logical reasoning, uncertainty management
                """,
                "page_reference": "Chapter 3, Page 36-58"
            }
        ]

        return sample_content