# Test the exact logic from the updated code with the same query
query = "What is AI?"
query_lower = query.lower()
query_words = query_lower.split()  # Split into individual words

print(f"Query: {query}")
print(f"Query words: {query_words}")

# Check for greeting words
greeting_found = False
for word in query_words:
    if word in ["hello", "hi", "hey", "greetings"]:
        greeting_found = True
        print(f"Found greeting word: {word}")
        break

print(f"Greeting found: {greeting_found}")

if greeting_found:
    print("Would return: Hello! I'm your AI assistant...")
else:
    # Check for question words
    question_found = False
    for word in query_words:
        if word in ["help", "what", "who", "how"]:
            question_found = True
            print(f"Found question word: {word}")
            break

    print(f"Question found: {question_found}")
    if question_found:
        print("Would return: I'd like to help with your question...")
    else:
        # Check for other words
        other_found = False
        for word in query_words:
            if word in ["name", "you"]:
                other_found = True
                print(f"Found other word: {word}")
                break

        print(f"Other found: {other_found}")
        if other_found:
            print("Would return: I'm an AI assistant...")
        else:
            print("Would return: I understand you asked...")