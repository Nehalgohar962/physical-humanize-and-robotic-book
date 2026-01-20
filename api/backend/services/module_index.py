"""
Module index for the Physical AI & Humanoid Robotics book
Provides structured access to module information for direct queries
"""
import re

MODULE_INDEX = {
    1: {
        "module_number": 1,
        "module_title": "Introduction to Humanoid Robots",
        "description": "Humanoid robots represent a fascinating intersection of mechanical engineering, artificial intelligence, and human psychology...",
        "topics": ["Humanoid design", "Robotics fundamentals", "AI integration"]
    },
    2: {
        "module_number": 2,
        "module_title": "AI Motion Planning",
        "description": "Effective motion planning in humanoid robots requires sophisticated algorithms that can navigate complex environments...",
        "topics": ["Motion planning", "Pathfinding algorithms", "Navigation systems"]
    },
    3: {
        "module_number": 3,
        "module_title": "Design Principles",
        "description": "The design of humanoid robots involves careful consideration of biomechanics, sensor integration, and control systems...",
        "topics": ["Biomechanics", "Sensor integration", "Control systems"]
    },
    4: {
        "module_number": 4,
        "module_title": "RAG-Based Knowledge Systems",
        "description": "Retrieval-Augmented Generation systems provide robots with access to extensive knowledge bases for informed decision making...",
        "topics": ["RAG systems", "Knowledge retrieval", "Information systems"]
    },
    5: {
        "module_number": 5,
        "module_title": "Human-Robot Interaction",
        "description": "Successful human-robot interaction requires intuitive interfaces and natural communication patterns...",
        "topics": ["Human-robot interaction", "Communication", "Interface design"]
    },
    6: {
        "module_number": 6,
        "module_title": "Ethical, Social & Future Challenges of Humanoid Robotics",
        "description": "The future of humanoid robotics holds promise for applications in healthcare, education, and industrial settings...",
        "topics": ["Ethics", "Social impact", "Future applications"]
    }
}

def get_module_info(module_number):
    """Get specific module information by number"""
    return MODULE_INDEX.get(module_number)

def get_all_modules():
    """Get all modules as a list"""
    return list(MODULE_INDEX.values())

def is_module_query(query):
    """Check if the query is asking about a specific module"""
    query_lower = query.lower().strip()

    # Check for patterns like "module X", "module number X", etc.
    import re
    module_patterns = [
        r'module\s+(\d+)',
        r'module\s+number\s+(\d+)',
        r'what is module (\d+)',
        r'what does module (\d+) cover',
        r'tell me about module (\d+)',
        r'what is the title of module (\d+)',
        r'module (\d+) name',
        r'module (\d+) title',
        r'list of modules',
        r'all modules'
    ]

    for pattern in module_patterns:
        match = re.search(pattern, query_lower)
        if match:
            return True, match.group(1) if match.groups() else None

    return False, None

def handle_module_query(query):
    """Handle module-specific queries directly - only for direct module information requests"""

    # Check for "all modules" or "list of modules" query first
    if any(phrase in query.lower() for phrase in ['all modules', 'list of modules', 'modules list', 'what modules', 'show modules']):
        modules_list = []
        for num in sorted(MODULE_INDEX.keys()):
            info = MODULE_INDEX[num]
            modules_list.append(f"""Module {num}: {info['module_title']}
Description: {info['description']}
Topics: {', '.join(info['topics'])}
""")

        response = "Here are all the modules in the Physical AI & Humanoid Robotics book:\n\n" + "\n\n".join(modules_list)
        references = [f"Module {num}: {MODULE_INDEX[num]['module_title']}" for num in sorted(MODULE_INDEX.keys())]
        return {
            "response": response,
            "references": references,
            "is_module_specific": True
        }

    # Only handle direct module queries (like "what is module 3?" or "module 3 name")
    # Use more restrictive patterns to avoid catching content questions
    is_module_q, module_num = is_module_query(query)

    # Check if it's a direct module query (asking for module title, name, or basic info)
    query_lower = query.lower().strip()
    direct_module_patterns = [
        r'what is module \d+\?',
        r'what does module \d+ cover',
        r'tell me about module \d+',
        r'module \d+ name',
        r'module \d+ title',
        r'module \d+ description',
        r'module \d+ info',
        r'module \d+ overview'
    ]

    is_direct_query = any(re.search(pattern, query_lower) for pattern in direct_module_patterns)

    if is_module_q and module_num and is_direct_query:
        try:
            module_num = int(module_num)
            module_info = get_module_info(module_num)
            if module_info:
                response = f"""Module {module_num}: {module_info['module_title']}

Description: {module_info['description']}

Key Topics: {', '.join(module_info['topics'])}"""
                references = [f"Module {module_num}: {module_info['module_title']}"]
                return {
                    "response": response,
                    "references": references,
                    "is_module_specific": True
                }
        except ValueError:
            pass

    # Return None for content-related queries that should go through RAG
    return None