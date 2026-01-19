# Test the logic
queries = ["What is AI?", "Hello", "Tell me about robotics", "How does AI work?", "Hello there"]

for query in queries:
    print(f"\nQuery: {query}")
    query_lower = query.lower()
    query_words = query_lower.split()
    print(f"Query words: {query_words}")

    # Test first condition
    condition1 = any(word in query_words for word in ["hello", "hi", "hey", "greetings"])
    print(f"Condition 1 (hello, hi, hey, greetings): {condition1}")

    # Test second condition
    condition2 = any(word in query_words for word in ["help", "what", "who", "how"])
    print(f"Condition 2 (help, what, who, how): {condition2}")

    # Test third condition
    condition3 = any(word in query_words for word in ["name", "you"])
    print(f"Condition 3 (name, you): {condition3}")

    if condition1:
        print("Would return: Hello! I'm your AI assistant...")
    elif condition2:
        print("Would return: I'd like to help with your question...")
    elif condition3:
        print("Would return: I'm an AI assistant...")
    else:
        print("Would return: I understand you asked...")