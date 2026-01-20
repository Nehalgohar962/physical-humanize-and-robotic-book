import requests
import json

# Base URL for the running API
BASE_URL = "http://localhost:8000"

def chat_with_bot():
    print("[ROBOT] RAG Chatbot is running!")
    print("Type 'quit' to exit the chat")
    print("="*50)

    # Step 1: Create a session
    print("Creating a new session...")
    session_response = requests.post(f"{BASE_URL}/api/session/sessions")
    if session_response.status_code != 200:
        print("Error: Could not create session")
        return

    session_data = session_response.json()
    session_id = session_data['id']
    print(f"Session created with ID: {session_id}")
    print()

    print("You can now chat with the RAG Chatbot!")
    print("Note: The bot has been loaded with textbook content about Physical AI & Humanoid Robotics")
    print()

    while True:
        user_input = input("You: ")
        if user_input.lower() in ['quit', 'exit', 'q']:
            print("Goodbye! 👋")
            break

        # Send message to the chatbot
        chat_payload = {
            "session_id": session_id,
            "message": user_input,
            "mode": "full_book"
        }

        try:
            chat_response = requests.post(f"{BASE_URL}/api/chat/chat", json=chat_payload)
            if chat_response.status_code == 200:
                response_data = chat_response.json()
                print(f"Bot: {response_data['response']}")
                if response_data.get('references'):
                    print(f"References: {response_data['references']}")
            else:
                print(f"Error: {chat_response.status_code} - {chat_response.text}")
        except Exception as e:
            print(f"Connection error: {e}")
            print("Make sure the server is running on http://localhost:8000")
            break

if __name__ == "__main__":
    print("Welcome to the RAG Chatbot for Physical AI & Humanoid Robotics! [ROBOT]")
    print("This chatbot has access to textbook content about AI and robotics.")
    print()
    chat_with_bot()