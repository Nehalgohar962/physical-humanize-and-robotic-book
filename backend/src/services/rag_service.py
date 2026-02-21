# import sys
# import os
# from typing import List, Dict, Any, Optional
# import random

# # Add the backend/src directory to the Python path to allow absolute imports
# sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# from config import Settings, settings
# from services.textbook_content_service import TextbookContentService
# from services.embedding_service import EmbeddingService
# from services.module_index import handle_module_query, get_all_modules


# class RAGService:
#     def __init__(self):
#         print("DEBUG: RAGService.__init__ called")
#         self.textbook_service = TextbookContentService()
#         self.embedding_service = EmbeddingService()
#         # Initialize the appropriate LLM client based on provider
#         self.client = self._initialize_client()

#     def _initialize_client(self):
#         """Initialize the appropriate LLM client based on the configured provider"""
#         if settings.llm_provider.lower() == "gemini":
#             try:
#                 import google.generativeai as genai
#                 genai.configure(api_key=settings.gemini_api_key)
#                 # Use a model that's more likely to be available
#                 return genai.GenerativeModel('gemini-2.5-flash')
#             except ImportError:
#                 raise ImportError("Please install google-generativeai: pip install google-generativeai")
#         elif settings.llm_provider.lower() == "openai":
#             try:
#                 from openai import OpenAI
#                 return OpenAI(api_key=settings.openai_api_key)
#             except ImportError:
#                 raise ImportError("Please install openai: pip install openai")
#         else:
#             raise ValueError(f"Unsupported LLM provider: {settings.llm_provider}")

#     def generate_response(
#         self,
#         query: str,
#         mode: str = "full_book",  # "full_book" or "context_only"
#         context_text: Optional[str] = None,
#         max_results: int = 5,
#         modules: List[int] = [1, 2, 3, 4, 5, 6]  # List of modules to search (default: all 6 modules)
#     ) -> Dict[str, Any]:
#         """
#         Generate a response using RAG based on the query
#         """
#         print(f"DEBUG: Starting generate_response for query: {query}")

#         # Check if this is a contextual query (contains conversation history)
#         is_contextual = "\nCurrent query:" in query or any(role in query.lower() for role in ["user:", "assistant:", "system:"])

#         # For contextual queries, we want to focus on the actual current question
#         # Extract the current query if it's in conversation format
#         current_query = query
#         original_query_lower = query.lower().strip()

#         if is_contextual:
#             # Extract the actual current query from the contextual format
#             if "\nCurrent query:" in query:
#                 current_query = query.split("\nCurrent query:")[-1].strip()
#             else:
#                 current_query = query

#             # For contextual queries, only check for greetings, not for module listings
#             # since those are typically initial queries, not follow-ups
#             greeting_words = ["hello", "hi", "hey", "greetings"]

#             # Check for greetings only in the current query, not the full contextual query
#             # Use whole word matching to avoid substring matches (e.g., "hi" in "this")
#             import re
#             current_query_lower = current_query.lower().strip()
#             if any(re.search(r'\b' + re.escape(greeting) + r'\b', current_query_lower) for greeting in greeting_words):
#                 print(f"DEBUG: Greeting detected in current query: {current_query}")
#                 # Return the proper greeting with module list
#                 modules_list = """Module 1: The Robotic Nervous System (ROS 2)
# Module 2: The Digital Twin (Gazebo & Unity)
# Module 3: The AI-Robot Brain (NVIDIA Isaac)
# Module 4: Vision-Language-Action (VLA)
# Module 5: Humanoid Robot Development
# Module 6: Conversational Robotics"""

#                 greeting_response = f"Hello! How can I help you today with Physical AI & Humanoid Robotics?\n\n{modules_list}"
#                 return {
#                     "response": greeting_response,
#                     "references": []
#                 }
#         else:
#             # For non-contextual (standalone) queries, apply all the original checks
#             # Check for greeting words to provide proper greeting behavior
#             query_lower = query.lower().strip()
#             print(f"DEBUG: Processing query_lower: {query_lower}")
#             greeting_words = ["hello", "hi", "hey", "greetings"]

#             import re
#             if any(re.search(r'\b' + re.escape(greeting) + r'\b', query_lower) for greeting in greeting_words):
#                 print(f"DEBUG: Greeting detected for query: {query}")
#                 # Return the proper greeting with module list
#                 modules_list = """Module 1: The Robotic Nervous System (ROS 2)
# Module 2: The Digital Twin (Gazebo & Unity)
# Module 3: The AI-Robot Brain (NVIDIA Isaac)
# Module 4: Vision-Language-Action (VLA)
# Module 5: Humanoid Robot Development
# Module 6: Conversational Robotics"""

#                 greeting_response = f"Hello! How can I help you today with Physical AI & Humanoid Robotics?\n\n{modules_list}"
#                 return {
#                     "response": greeting_response,
#                     "references": []
#                 }

#             # Check if this is a direct modules listing query (only for explicit requests)
#             if any(phrase in query_lower for phrase in ['all modules', 'list of modules', 'modules list', 'what modules', 'show modules']) and \
#                any(word in query_lower.split() for word in ['modules', 'module', 'list', 'show', 'what']):
#                 print(f"DEBUG: Direct modules listing query detected: {query}")
#                 modules_list = """Module 1: The Robotic Nervous System (ROS 2)
# Module 2: The Digital Twin (Gazebo & Unity)
# Module 3: The AI-Robot Brain (NVIDIA Isaac)
# Module 4: Vision-Language-Action (VLA)
# Module 5: Humanoid Robot Development
# Module 6: Conversational Robotics"""

#                 modules_response = f"Here are the book modules:\n\n{modules_list}"
#                 return {
#                     "response": modules_response,
#                     "references": []
#                 }

#         print(f"DEBUG: No special handling for query: {query}, proceeding with RAG search")

#         try:
#             # Get relevant context based on mode
#             if mode == "context_only" and context_text:
#                 # Use only the provided context
#                 relevant_texts = [context_text]
#                 references = ["Provided context"]
#             else:
#                 # For contextual queries, use the extracted current query for module detection and search
#                 search_query = current_query if is_contextual else query

#                 # Determine if the query is about a specific module
#                 detected_module_id = self._detect_module_in_query(search_query)

#                 # Check if user explicitly wants a specific module
#                 is_specific_request = detected_module_id and any(phrase in search_query.lower() for phrase in ['specifically', 'just module', 'only module', 'just the', 'only the module'])

#                 if is_specific_request:
#                     # Do targeted search if user explicitly wants a specific module
#                     print(f"DEBUG: Detected specific module request {detected_module_id}, using targeted search")
#                     search_results = self.textbook_service.search_content_by_module(
#                         search_query, detected_module_id, limit=max_results
#                     )
#                     print(f"DEBUG: Found {len(search_results)} results from module-specific search")
#                 else:
#                     # Use the modules parameter to determine which modules to search
#                     print(f"DEBUG: Using modules {modules} for search (all modules 1-6 unless specified)")

#                     # If modules are specified, search only those modules; otherwise, search all content
#                     if modules and len(modules) > 0 and modules != [1, 2, 3, 4, 5, 6]:
#                         # Search specific modules only
#                         all_search_results = []

#                         # Map module numbers to their identifiers
#                         module_map = {
#                             1: ('module1-introduction', 'Module 1: The Robotic Nervous System'),
#                             2: ('module2-digital-twin', 'Module 2: The Digital Twin'),
#                             3: ('module3-ai-brain', 'Module 3: The AI-Robot Brain'),
#                             4: ('module4-vla', 'Module 4: Vision-Language-Action'),
#                             5: ('module5-humanoid-design', 'Module 5: Humanoid Robot Development'),
#                             6: ('module6-conversational', 'Module 6: Conversational Robotics')
#                         }

#                         for module_num in modules:
#                             if module_num in module_map:
#                                 module_id_part, module_name = module_map[module_num]
#                                 try:
#                                     module_results = self.textbook_service.search_content_by_module(
#                                         search_query, module_id_part, limit=max_results//len(modules)  # Distribute limit across modules
#                                     )
#                                     print(f"DEBUG: Found {len(module_results)} results from {module_name}")
#                                     all_search_results.extend(module_results)
#                                 except Exception as e:
#                                     print(f"DEBUG: Error searching {module_name}: {e}")
#                                     # Continue with other modules even if one fails

#                         search_results = all_search_results
#                     else:
#                         # Search all content to ensure comprehensive coverage of all modules
#                         print(f"DEBUG: Using comprehensive search across all content (all modules 1-6)")

#                         # Do a general search that covers all content
#                         search_results = self.textbook_service.search_content(
#                             search_query, limit=max_results
#                         )
#                         print(f"DEBUG: Found {len(search_results)} results from general search")

#                     # If no results from module-specific or general search, try fallback
#                     if not search_results:
#                         print(f"DEBUG: No results from search, trying fallback search")
#                         search_results = self.textbook_service.search_content_fallback(
#                             search_query, limit=max_results
#                         )
#                         print(f"DEBUG: Found {len(search_results)} results from fallback search")

#                 print(f"DEBUG: Total search results count: {len(search_results)}")

#                 # Ensure we ALWAYS have results - if still no results after comprehensive search, get ANY content
#                 if not search_results:
#                     print(f"DEBUG: No results from comprehensive search, trying fallback search")
#                     search_results = self.textbook_service.search_content_fallback(
#                         search_query, limit=max_results
#                     )
#                     print(f"DEBUG: Found {len(search_results)} results from fallback search")

#                     # If STILL no results, force at least one piece of content from the database
#                     if not search_results:
#                         print(f"DEBUG: No content found, this is unexpected - forcing general search to collection")
#                         try:
#                             # Search without filters as a last resort
#                             from services.vector_db_service import VectorDBService
#                             temp_vector_db = VectorDBService()
#                             temp_embedding = self.embedding_service.embed(search_query)

#                             # Try to get ANY content from the textbook_content collection
#                             any_content_results = temp_vector_db.search_similar(
#                                 query_embedding=temp_embedding,
#                                 collection_name="textbook_content",
#                                 limit=max_results
#                             )
#                             print(f"DEBUG: Found {len(any_content_results)} results from direct collection search")
#                             search_results = any_content_results
#                         except Exception as e:
#                             print(f"DEBUG: Direct collection search also failed: {e}")
#                             # As ultimate fallback, create an empty result so we don't fail
#                             search_results = [{
#                                 "content_text": "Placeholder content for query: " + search_query,
#                                 "page_reference": "general_reference",
#                                 "similarity_score": 0.0
#                             }]

#                 # Extract content and references from search results
#                 relevant_texts = []
#                 references = []

#                 for result in search_results:
#                     if isinstance(result, dict):
#                         content_text = result.get("content_text", "")
#                         if content_text.strip():  # Only add non-empty content
#                             relevant_texts.append(content_text)
#                             references.append(result.get("page_reference", "General Reference"))
#                     else:
#                         # Handle case where result might be a different format
#                         print(f"DEBUG: Unexpected result format: {type(result)} - {result}")
#                         if hasattr(result, 'content_text'):
#                             content_text = getattr(result, 'content_text', '')
#                             if content_text.strip():  # Only add non-empty content
#                                 relevant_texts.append(content_text)
#                                 references.append(getattr(result, 'page_reference', 'General Reference'))

#                 print(f"DEBUG: Final relevant_texts count: {len(relevant_texts)}")
#                 print(f"DEBUG: Final references count: {len(references)}")
#                 if relevant_texts:
#                     print(f"DEBUG: First relevant text preview: {relevant_texts[0][:100]}...")

#             # Prepare the prompt for the language model
#             # Use the current query for the question to ensure clarity for the LLM
#             display_query = current_query if is_contextual else query

#             # Include conversation history in the prompt if available
#             conversation_context = ""
#             if is_contextual and "\n".join([part for part in query.split("\n") if "Current query:" not in part]):  # If there's conversation history in the query
#                 # Extract the conversation history part (before "Current query:")
#                 full_query_parts = query.split("\nCurrent query:")
#                 if len(full_query_parts) > 1:
#                     conversation_context = full_query_parts[0].strip()
#                     if conversation_context:
#                         conversation_context = f"\n\nPrevious conversation context:\n{conversation_context}"

#             if relevant_texts:
#                 context_str = "\n\n".join(relevant_texts)
#                 prompt = f"""
#                 Answer the following question based ONLY on the provided context from the Physical AI & Humanoid Robotics textbook.

#                 Context:
#                 {context_str}{conversation_context}

#                 Question: {display_query}

#                 STRICT INSTRUCTIONS:
#                 - Answer based ONLY on the provided textbook content
#                 - Do NOT use general AI knowledge
#                 - Use the information from the context to provide the best possible answer
#                 - If the exact information is not available, provide related information from the context
#                 - Be comprehensive and helpful using ONLY the provided context
#                 - If citing specific information, mention the section titles or page references
#                 - If multiple modules are involved, combine them in one clear answer
#                 - If the question matches a specific module, clearly mention the module name
#                 - Use clear academic language that is easy to understand
#                 - End every answer with a "References from Book" section
#                 - References must list the correct module(s) and section titles from the provided context
#                 - DO NOT respond with "This topic is not covered in the current textbook modules" if there is any relevant information in the context
#                 - Use the conversation context to maintain topic continuity and provide relevant follow-up responses
#                 - Follow the response format exactly as specified

#                 Response Format:
#                 [Clear explanation in simple academic language based on textbook content]

#                 References from Book:
#                 - [Module name and section title from context]
#                 - [Additional modules/sections as relevant]
#                 """
#             else:
#                 prompt = f"""
#                 The following question was asked: {display_query}{conversation_context}

#                 This topic is not covered in the current textbook modules.

#                 Response Format:
#                 This topic is not covered in the current textbook modules.

#                 References from Book:
#                 - No relevant modules found
#                 """

#             # Generate response using the appropriate LLM provider
#             if settings.llm_provider.lower() == "gemini":
#                 # For Gemini, we need to combine system and user messages
#                 # IMPORTANT: Modify the system prompt to ensure that if context exists, it's always used
#                 if relevant_texts:  # If we have relevant texts, instruct to use them
#                     full_prompt = f"""You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based ONLY on the provided textbook content. Do NOT give generic AI answers.

# IMPORTANT: Since I have provided textbook content in the context, you MUST use this information to answer the question. Do not respond with "This topic is not covered in the current textbook modules" if there is any relevant information in the provided context.

# MODULE-BASED ANSWERING INSTRUCTIONS:
# - Use all 6 modules if relevant to the question
# - If the question matches a specific module, clearly mention the module name (Module 1: The Robotic Nervous System, Module 2: The Digital Twin, etc.)
# - If multiple modules are involved, combine them in one clear answer
# - Always answer strictly based on the book content only
# - Use clear explanation in simple academic language
# - End every answer with a "References from Book" section
# - References must list the correct module(s) and section titles

# {prompt}"""
#                 else:  # If no relevant texts, then it's appropriate to say it's not covered
#                     full_prompt = f"""You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based ONLY on the provided textbook content. Do NOT give generic AI answers. If the content doesn't contain the information needed to answer the question, respond with: "This topic is not covered in the current textbook modules."

# MODULE-BASED ANSWERING INSTRUCTIONS:
# - Use all 6 modules if relevant to the question
# - If the question matches a specific module, clearly mention the module name (Module 1: The Robotic Nervous System, Module 2: The Digital Twin, etc.)
# - If multiple modules are involved, combine them in one clear answer
# - Always answer strictly based on the book content only
# - Use clear explanation in simple academic language
# - End every answer with a "References from Book" section
# - References must list the correct module(s) and section titles

# {prompt}"""

#                 try:
#                     response = self.client.generate_content(
#                         full_prompt,
#                         generation_config={
#                             "max_output_tokens": 1000,
#                             "temperature": 0.1,  # Lower temperature for more consistent responses
#                         }
#                     )

#                     # Extract text from the response safely
#                     answer = "This topic is not covered in the current textbook modules."
#                     try:
#                         if hasattr(response, 'text') and response.text:
#                             answer = response.text.strip()
#                         elif (hasattr(response, '_result') and
#                               hasattr(response._result, 'candidates') and
#                               response._result.candidates and
#                               hasattr(response._result.candidates[0], 'content') and
#                               hasattr(response._result.candidates[0].content, 'parts') and
#                               response._result.candidates[0].content.parts):
#                             answer = response._result.candidates[0].content.parts[0].text.strip()
#                         # If we have relevant texts but the response is still a "not covered" message,
#                         # create a response using the context
#                         if relevant_texts and ("not covered" in answer.lower() or "not yet covered" in answer.lower()):
#                             print(f"DEBUG: LLM returned 'not covered' despite having context, creating response from context")
#                             context_preview = "\n".join(relevant_texts[:2])  # Use first 2 relevant texts
#                             answer = f"Based on the textbook content:\n\n{context_preview}\n\nFor more details, see the referenced sections."
#                     except (IndexError, AttributeError, TypeError):
#                         # If any part of the response extraction fails, create a response using the available context
#                         if relevant_texts:
#                             print(f"DEBUG: LLM response extraction failed, creating response from context")
#                             context_preview = "\n".join(relevant_texts[:2])  # Use first 2 relevant texts
#                             answer = f"Based on the textbook content:\n\n{context_preview}\n\nFor more details, see the referenced sections."
#                         else:
#                             pass  # Keep the default answer if no context available
#                 except Exception as e:
#                     # If the API call fails completely (quota, rate limit, etc.), create a response from the available content
#                     print(f"DEBUG: Gemini API call failed: {e}")
#                     if relevant_texts:
#                         print(f"DEBUG: Creating response from available context")
#                         context_preview = "\n".join(relevant_texts[:3])  # Use first 3 relevant texts for more content
#                         answer = f"Based on the textbook content:\n\n{context_preview}\n\nReferences from Book:\n" + "\n".join([f"- {ref}" for ref in references[:5]])
#                     else:
#                         answer = "This topic is not covered in the current textbook modules."

#             elif settings.llm_provider.lower() == "openai":
#                 # Generate response using OpenAI
#                 # IMPORTANT: Modify system message based on whether we have relevant texts
#                 if relevant_texts:  # If we have relevant texts, instruct to use them
#                     system_message = """You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based ONLY on the provided textbook content. Do NOT give generic AI answers.

# IMPORTANT: Since I have provided textbook content in the context, you MUST use this information to answer the question. Do not respond with "This topic is not covered in the current textbook modules" if there is any relevant information in the provided context.

# MODULE-BASED ANSWERING INSTRUCTIONS:
# - Use all 6 modules if relevant to the question
# - If the question matches a specific module, clearly mention the module name (Module 1: The Robotic Nervous System, Module 2: The Digital Twin, etc.)
# - If multiple modules are involved, combine them in one clear answer
# - Always answer strictly based on the book content only
# - Use clear explanation in simple academic language
# - End every answer with a "References from Book" section
# - References must list the correct module(s) and section titles"""
#                 else:  # If no relevant texts, then it's appropriate to say it's not covered
#                     system_message = """You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based ONLY on the provided textbook content. Do NOT give generic AI answers. If the content doesn't contain the information needed to answer the question, respond with: "This topic is not covered in the current textbook modules."

# MODULE-BASED ANSWERING INSTRUCTIONS:
# - Use all 6 modules if relevant to the question
# - If the question matches a specific module, clearly mention the module name (Module 1: The Robotic Nervous System, Module 2: The Digital Twin, etc.)
# - If multiple modules are involved, combine them in one clear answer
# - Always answer strictly based on the book content only
# - Use clear explanation in simple academic language
# - End every answer with a "References from Book" section
# - References must list the correct module(s) and section titles"""

#                 try:
#                     response = self.client.chat.completions.create(
#                         model="gpt-3.5-turbo",
#                         messages=[
#                             {"role": "system", "content": system_message},
#                             {"role": "user", "content": prompt}
#                         ],
#                         max_tokens=1000,
#                         temperature=0.1  # Lower temperature for more consistent responses
#                     )

#                     # Safely extract OpenAI response
#                     try:
#                         answer = response.choices[0].message.content.strip()
#                         # If we have relevant texts but the response is still a "not covered" message,
#                         # create a response using the context
#                         if relevant_texts and ("not covered" in answer.lower() or "not yet covered" in answer.lower()):
#                             print(f"DEBUG: OpenAI returned 'not covered' despite having context, creating response from context")
#                             context_preview = "\n".join(relevant_texts[:2])  # Use first 2 relevant texts
#                             answer = f"Based on the textbook content:\n\n{context_preview}\n\nFor more details, see the referenced sections."
#                     except (IndexError, AttributeError, TypeError):
#                         # If response extraction fails, create a response using the available context
#                         if relevant_texts:
#                             print(f"DEBUG: OpenAI response extraction failed, creating response from context")
#                             context_preview = "\n".join(relevant_texts[:2])  # Use first 2 relevant texts
#                             answer = f"Based on the textbook content:\n\n{context_preview}\n\nFor more details, see the referenced sections."
#                         else:
#                             answer = "This topic is not covered in the current textbook modules."
#                 except Exception as e:
#                     # If the API call fails completely (quota, rate limit, etc.), create a response from the available content
#                     print(f"DEBUG: OpenAI API call failed: {e}")
#                     if relevant_texts:
#                         print(f"DEBUG: Creating response from available context")
#                         context_preview = "\n".join(relevant_texts[:3])  # Use first 3 relevant texts for more content
#                         answer = f"Based on the textbook content:\n\n{context_preview}\n\nReferences from Book:\n" + "\n".join([f"- {ref}" for ref in references[:5]])
#                     else:
#                         answer = "This topic is not covered in the current textbook modules."
#             else:
#                 raise ValueError(f"Unsupported LLM provider: {settings.llm_provider}")

#             return {
#                 "response": answer,
#                 "references": references if relevant_texts else []
#             }
#         except Exception as e:
#             import traceback
#             error_msg = str(e)
#             print(f"Error generating RAG response: {error_msg}")
#             print(f"Full traceback: {traceback.format_exc()}")

#             # Check for specific errors and provide more detailed guidance based on provider
#             if "authentication" in error_msg.lower() or "api key" in error_msg.lower() or "invalid_api_key" in error_msg.lower():
#                 if settings.llm_provider.lower() == "gemini":
#                     detailed_msg = "Authentication error: Your Google Gemini API key is invalid. Please verify that your API key in the .env file is correct and properly formatted."
#                 else:
#                     detailed_msg = "Authentication error: Your OpenAI API key is invalid. Please verify that your API key in the .env file is correct and properly formatted."
#             elif "quota" in error_msg.lower() or "credit" in error_msg.lower() or "insufficient_quota" in error_msg.lower():
#                 if settings.llm_provider.lower() == "gemini":
#                     detailed_msg = "Quota exceeded: Your Google Gemini API key has reached its usage limit. Please check your Google AI Studio billing and usage."
#                 else:
#                     detailed_msg = "Quota exceeded: Your OpenAI API key has reached its usage limit. Please check your OpenAI billing and usage at https://platform.openai.com/account/billing/overview. You may need to add funds to your account or use a different API key with available quota."
#             elif "rate limit" in error_msg.lower() or "rate_limit" in error_msg.lower():
#                 if settings.llm_provider.lower() == "gemini":
#                     detailed_msg = "Rate limit exceeded: Too many requests to Google Gemini. Please try again later."
#                 else:
#                     detailed_msg = "Rate limit exceeded: Too many requests to OpenAI. Please try again later or check your rate limits at https://platform.openai.com/account/rate-limits"
#             elif "model" in error_msg.lower():
#                 if settings.llm_provider.lower() == "gemini":
#                     detailed_msg = f"Model error: {error_msg}. Please check if the model 'gemini-2.5-flash' is available in your Google AI account."
#                 else:
#                     detailed_msg = f"Model error: {error_msg}. Please check if the model 'gpt-3.5-turbo' is available in your OpenAI account."
#             elif "access" in error_msg.lower() or "permission" in error_msg.lower():
#                 if settings.llm_provider.lower() == "gemini":
#                     detailed_msg = f"Access error: {error_msg}. Your Google AI account may not have access to the requested model or service."
#                 else:
#                     detailed_msg = f"Access error: {error_msg}. Your OpenAI account may not have access to the requested model or service."
#             else:
#                 detailed_msg = f"Error: {error_msg}. Please check your {settings.llm_provider.upper()} API configuration and billing status."

#             # For error handling, check if this is a contextual query
#             greeting_words = ["hello", "hi", "hey", "greetings"]

#             # Check for greetings only in the current query, not the full contextual query
#             # Use whole word matching to avoid substring matches (e.g., "hi" in "this")
#             import re
#             current_query_lower = current_query.lower().strip()
#             if any(re.search(r'\b' + re.escape(greeting) + r'\b', current_query_lower) for greeting in greeting_words):
#                 # Return the proper greeting with module list
#                 modules_list = """Module 1: The Robotic Nervous System (ROS 2)
# Module 2: The Digital Twin (Gazebo & Unity)
# Module 3: The AI-Robot Brain (NVIDIA Isaac)
# Module 4: Vision-Language-Action (VLA)
# Module 5: Humanoid Robot Development
# Module 6: Conversational Robotics"""

#                 basic_response = f"Hello! How can I help you today with Physical AI & Humanoid Robotics?\n\n{modules_list}"
#             else:
#                 # For contextual queries during error handling, avoid showing generic module lists
#                 # Check if this looks like a modules-related query but avoid false positives in conversation context
#                 if not is_contextual and ("module" in current_query_lower or "modules" in current_query_lower or "name" in current_query_lower):
#                     modules_list = """Module 1: The Robotic Nervous System (ROS 2)
# Module 2: The Digital Twin (Gazebo & Unity)
# Module 3: The AI-Robot Brain (NVIDIA Isaac)
# Module 4: Vision-Language-Action (VLA)
# Module 5: Humanoid Robot Development
# Module 6: Conversational Robotics"""

#                     basic_response = f"Here are the book modules:\n\n{modules_list}"
#                 elif "book" in current_query_lower or "content" in current_query_lower:
#                     basic_response = "I can help you with information from the Physical AI & Humanoid Robotics book. The book covers 6 main modules including The Robotic Nervous System, Advanced AI Motion Control, Humanoid Robot Design Principles, Embodied Intelligence Systems, Control Systems for Humanoid Robots, and Ethical and Social Implications. What would you like to know?"
#                 else:
#                     # Check if we have content in our database despite API issues
#                     # Use the current query for contextual searches during error handling
#                     search_error_query = current_query if is_contextual else query
#                     try:
#                         search_results = self.textbook_service.search_content(search_error_query, limit=3)
#                         if search_results:
#                             # We have content, but API is down - provide a graceful error message
#                             basic_response = "The service is temporarily unavailable. Please try again in a moment."
#                         else:
#                             # No content found and API is down
#                             basic_response = "This topic is not yet covered in the book."
#                     except:
#                         # If search also fails, return a generic response
#                         basic_response = "This topic is not yet covered in the book."

#             return {
#                 "response": basic_response,
#                 "references": []
#             }

#     def _detect_module_in_query(self, query: str) -> Optional[str]:
#         """
#         Detect if the query is about a specific module and return the module identifier
#         """
#         import re  # Import once at the beginning
#         query_lower = query.lower().strip()

#         # Define patterns to detect module references
#         module_patterns = [
#             # Module number patterns - these take precedence
#             (r'module\s+1|first\s+module|module\s+one', 'module1-introduction'),
#             (r'module\s+2|second\s+module|module\s+two', 'module2-digital-twin'),
#             (r'module\s+3|third\s+module|module\s+three', 'module3-ai-brain'),
#             (r'module\s+4|fourth\s+module|module\s+four', 'module4-vla'),
#             (r'module\s+5|fifth\s+module|module\s+five', 'module5-humanoid-design'),
#             (r'module\s+6|sixth\s+module|module\s+six', 'module6-conversational'),
#             # Module topic patterns based on known module titles
#             (r'robotic.*nervous|ros.*2|nervous.*system', 'module1-introduction'),
#             (r'digital.*twin|gazebo|unity', 'module2-digital-twin'),
#             (r'ai.*robot.*brain|nvidia.*isaac', 'module3-ai-brain'),
#             (r'vision.*language.*action|vla', 'module4-vla'),
#             (r'humanoid.*robot.*develop|humanoid.*development', 'module5-humanoid-design'),
#             (r'conversational.*robot|dialogue|conversation', 'module6-conversational'),
#         ]

#         # Check each pattern
#         for pattern, module_id in module_patterns:
#             try:
#                 if re.search(pattern, query_lower):
#                     print(f"DEBUG: Detected {module_id} from pattern: {pattern}")
#                     return module_id
#             except re.error:
#                 # If there's a regex error, continue to the next pattern
#                 print(f"DEBUG: Regex error with pattern: {pattern}")
#                 continue

#         # Additional check for general module references
#         # If the query mentions a specific module number without exact patterns
#         try:
#             module_num_match = re.search(r'module\s+(\d+)', query_lower)
#             if module_num_match:
#                 module_num = module_num_match.group(1)
#                 module_map = {
#                     '1': 'module1-introduction',
#                     '2': 'module2-digital-twin',
#                     '3': 'module3-ai-brain',
#                     '4': 'module4-vla',
#                     '5': 'module5-humanoid-design',
#                     '6': 'module6-conversational'
#                 }
#                 if module_num in module_map:
#                     print(f"DEBUG: Detected module {module_num} from number pattern")
#                     return module_map[module_num]
#         except re.error:
#             # If there's a regex error, continue without detecting a module
#             print(f"DEBUG: Regex error in module number detection for query: {query_lower}")
#             pass

#         # If no specific module detected, return None
#         return None

#     def validate_query(self, query: str) -> bool:
#         """
#         Validate if the query is appropriate for the RAG system
#         """
#         if not query or len(query.strip()) < 1:
#             return False
#         return True

#     def get_conversation_context(
#         self,
#         query: str,
#         history: List[Dict[str, str]],
#         max_context_length: int = 2000
#     ) -> str:
#         """
#         Prepare conversation context by including recent history
#         """
#         if not history:
#             return query

#         # Build context from recent conversation history
#         context_parts = []
#         current_length = 0

#         # Add recent messages in reverse chronological order
#         for message in reversed(history):
#             message_text = f"{message['role']}: {message['content']}"
#             message_length = len(message_text)

#             if current_length + message_length > max_context_length:
#                 break

#             context_parts.insert(0, message_text)
#             current_length += message_length

#         if context_parts:
#             context = "\n".join(context_parts) + f"\n\nCurrent query: {query}"
#             return context
#         else:
#             return query

#     def test_llm_connection(self) -> Dict[str, Any]:
#         """
#         Test the LLM API connection with a simple request
#         """
#         try:
#             if settings.llm_provider.lower() == "gemini":
#                 response = self.client.generate_content(
#                     "Hello, this is a test. Respond with 'Test successful.'",
#                     generation_config={
#                         "max_output_tokens": 20,
#                         "temperature": 0.1,
#                     }
#                 )

#                 # Handle the response safely without triggering the text accessor error
#                 try:
#                     # Try to access text - this might raise an exception if finish_reason != 1
#                     if hasattr(response, 'text') and response.text:
#                         return {
#                             "success": True,
#                             "model": self.client.model_name,
#                             "message": "Google Gemini API connection successful"
#                         }
#                 except Exception:
#                     # If text accessor fails, check the result directly
#                     pass

#                 # Access the result directly to check finish_reason
#                 if hasattr(response, '_result') and response._result.candidates:
#                     candidate = response._result.candidates[0]
#                     if candidate.finish_reason == 1:  # STOP - normal completion
#                         return {
#                             "success": True,
#                             "model": self.client.model_name,
#                             "message": "Google Gemini API connection successful"
#                         }
#                     elif candidate.finish_reason in [2, 3, 4, 5]:  # SAFETY, RECITATION, OTHER, STOP
#                         # Some of these might still be valid responses, but blocked for various reasons
#                         if candidate.finish_reason == 2:  # SAFETY - blocked by safety settings
#                             # This means API is working but content was blocked - which is still a success
#                             return {
#                                 "success": True,
#                                 "model": self.client.model_name,
#                                 "message": "Google Gemini API connection successful (content blocked by safety settings)"
#                             }
#                         else:
#                             return {
#                                 "success": False,
#                                 "error": f"Gemini API returned finish_reason: {candidate.finish_reason} ({candidate.finish_message if hasattr(candidate, 'finish_message') else 'no message'})"
#                             }
#                     else:
#                         return {
#                             "success": True,
#                             "model": self.client.model_name,
#                             "message": "Google Gemini API connection successful"
#                         }
#                 else:
#                     return {
#                         "success": False,
#                         "error": "No response from Google Gemini API"
#                     }
#             elif settings.llm_provider.lower() == "openai":
#                 response = self.client.chat.completions.create(
#                     model="gpt-3.5-turbo",
#                     messages=[
#                         {"role": "system", "content": "You are a test assistant."},
#                         {"role": "user", "content": "Test connection"}
#                     ],
#                     max_tokens=10,
#                     temperature=0.0
#                 )

#                 if response.choices and len(response.choices) > 0:
#                     return {
#                         "success": True,
#                         "model": response.model,
#                         "message": "OpenAI API connection successful"
#                     }
#                 else:
#                     return {
#                         "success": False,
#                         "error": "No response from OpenAI API"
#                     }
#             else:
#                 return {
#                     "success": False,
#                     "error": f"Unsupported LLM provider: {settings.llm_provider}"
#                 }
#         except Exception as e:
#             error_msg = str(e)
#             return {
#                 "success": False,
#                 "error": error_msg,
#                 "error_type": f"{settings.llm_provider.upper()} API Error"
#             }


# rag_service.py

from services.module_index import handle_module_query, get_all_modules
from services.textbook_content_service import TextbookContentService


class RAGService:

    def __init__(self):
        print("✅ RAG Service Started...")

        # module info
        self.modules = get_all_modules()

        # REAL RAG engine
        self.textbook_service = TextbookContentService()

    # =====================================================
    # SMART MODULE SEARCH (fallback only)
    # =====================================================
    def _find_best_module(self, query: str):

        query_words = set(query.lower().split())
        best_match = None
        best_score = 0

        for module in self.modules:

            text = (
                module["module_title"] + " " +
                module["description"] + " " +
                " ".join(module["topics"])
            ).lower()

            module_words = set(text.split())

            score = len(query_words.intersection(module_words))

            if score > best_score:
                best_score = score
                best_match = module

        return best_match if best_score > 0 else None

    # =====================================================
    # MAIN RESPONSE FUNCTION
    # =====================================================
    def generate_response(self, query: str):

        print("\n==============================")
        print("User Query:", query)
        print("==============================\n")

        # =====================================================
        # STEP 1 — DIRECT MODULE QUESTION
        # =====================================================
        module_result = handle_module_query(query)

        if module_result and module_result.get("is_module_specific"):
            print("✅ Direct module info question")
            return {
                "query": query,
                "response": module_result["response"],
                "references": module_result.get("references", [])
            }

        # =====================================================
        # STEP 2 — REAL TEXTBOOK RAG
        # =====================================================
        print("📚 Searching textbook knowledge...")

        textbook_answer = self.textbook_service.answer_question(query)

        if textbook_answer and "could not find" not in textbook_answer.lower():
            print("✅ Answer from REAL textbook RAG")

            return {
                "query": query,
                "response": textbook_answer,
                "references": ["Textbook Knowledge Base"]
            }

        # =====================================================
        # STEP 3 — MODULE CONTENT FALLBACK
        # =====================================================
        print("📘 Fallback to module summary...")

        matched_module = self._find_best_module(query)

        if matched_module:
            response_text = f"""
Based on module overview:

Module {matched_module['module_number']}: {matched_module['module_title']}

{matched_module['description']}

Key Topics:
{', '.join(matched_module['topics'])}
"""

            return {
                "query": query,
                "response": response_text.strip(),
                "references": [f"Module {matched_module['module_number']}"]
            }

        # =====================================================
        # STEP 4 — NOTHING FOUND
        # =====================================================
        print("❌ Nothing found anywhere")

        return {
            "query": query,
            "response": "I could not find the answer in the textbook.",
            "references": []
        }


# =====================================================
# TERMINAL MODE
# =====================================================
if __name__ == "__main__":

    rag_service = RAGService()

    print("\n🤖 RAG Chatbot Ready!")
    print("Type 'exit' to quit.\n")

    while True:

        user_query = input("Enter your question: ")

        if user_query.lower() == "exit":
            print("👋 Goodbye!")
            break

        response = rag_service.generate_response(user_query)

        print("\n===== BOT RESPONSE =====")
        print(response["response"])
        print("References:", response["references"])
        print("\n")

