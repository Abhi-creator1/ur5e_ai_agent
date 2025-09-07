#!/usr/bin/env python3
"""
LLM Configuration for Ollama via OpenAI-compatible API
Uses ChatOpenAI from langchain-openai for proper tool calling support.
Switch between:
  1. Remote network host (current)
  2. Localhost Ollama (commented out)
  3. OpenAI API (commented out)
"""
from langchain_openai import ChatOpenAI

# -----------------------------------------------------------------------------
# 1) Remote network host (active)
# -----------------------------------------------------------------------------
def get_ollama_llm(model_name: str = "qwen2.5:14b", streaming: bool = True):
    """
    Configure ChatOpenAI LLM to use Ollama on a networked host.
    """
    llm = ChatOpenAI(
        model=model_name,
        streaming=streaming,
        temperature=0.1,
        openai_api_base="http://192.168.1.102:11434/v1",
        openai_api_key="OLLAMA"
    )
    print(f"🧠 Configured ChatOpenAI (Ollama network) LLM: {model_name}")
    return llm

def check_ollama_connection():
    """Check if the Ollama server on the network host is running."""
    try:
        import requests
        response = requests.get("http://192.168.1.102:11434/api/version", timeout=5)
        if response.status_code == 200:
            print("✅ Ollama network server is running")
            return True
    except Exception:
        pass
    print("❌ Ollama network server not accessible. Start with: ollama serve")
    return False

# -----------------------------------------------------------------------------
# 2) Localhost Ollama (uncomment to use)
# -----------------------------------------------------------------------------
"""
def get_ollama_llm(model_name: str = "qwen2.5:14b", streaming: bool = True):
    llm = ChatOpenAI(
        model=model_name,
        streaming=streaming,
        temperature=0.1,
        openai_api_base="http://localhost:11434/v1",
        openai_api_key="OLLAMA"
    )
    print(f"🧠 Configured ChatOpenAI (Ollama localhost) LLM: {model_name}")
    return llm

def check_ollama_connection():
    try:
        import requests
        response = requests.get("http://localhost:11434/api/version", timeout=5)
        if response.status_code == 200:
            print("✅ Ollama localhost server is running")
            return True
    except Exception:
        pass
    print("❌ Ollama localhost server not accessible. Start with: ollama serve")
    return False
"""

# -----------------------------------------------------------------------------
# 3) OpenAI Hosted API (uncomment to use)
# -----------------------------------------------------------------------------
"""
import os

def get_openai_llm(model_name: str = "gpt-4o", streaming: bool = True):
    llm = ChatOpenAI(
        model=model_name,
        streaming=streaming,
        temperature=0.0,
        openai_api_key=os.getenv("OPENAI_API_KEY"),
        openai_api_base="https://api.openai.com/v1"
    )
    print(f"🧠 Configured ChatOpenAI (OpenAI API) LLM: {model_name}")
    return llm

def check_openai_connection():
    try:
        import openai
        openai.api_key = os.getenv("OPENAI_API_KEY")
        _ = openai.Engine.list()
        print("✅ OpenAI API is reachable")
        return True
    except Exception:
        print("❌ OpenAI API not accessible. Check your API key and network.")
        return False
"""
