#!/usr/bin/env python3
"""
UR5e ROSA Agent - Main AI agent for natural language robot control
Leverages ROSA's internal tool-binding and agent executor for UR5e.
"""
import sys
import os
import asyncio
import rclpy
from rosa.rosa import ROSA
from prompts import get_ur5e_prompts
from llm_config import get_ollama_llm, check_ollama_connection
from rich.console import Console
from rich.panel import Panel
from rich.text import Text
import pyinputplus as pyip

# Import tools module as a package (like the reference project)
import tools.ur5e_control as ur5e_tools

class UR5eROSAAgent(ROSA):
    def __init__(self, streaming: bool = True, verbose: bool = True):
        """Initialize UR5e ROSA Agent using ROSA's internal tool binding."""
        
        # Check Ollama connection
        if not check_ollama_connection():
            raise ConnectionError("Ollama not accessible")
        
        # Get LLM and prompts
        llm = get_ollama_llm(streaming=streaming)
        prompts = get_ur5e_prompts()
        
        # Bind tools to LLM (but don't pass explicit tools list)
        # Let ROSA discover them from tool_packages
        llm = llm.bind_tools([])  # Empty list, let tool_packages handle discovery
        
        # Initialize ROSA with tool packages (like reference project)
        super().__init__(
            ros_version=2,
            llm=llm,
            tools=[],  # Empty - let tool_packages discover
            tool_packages=[ur5e_tools],  # Pass the actual module object
            prompts=prompts,
            verbose=verbose,
            streaming=streaming,
            accumulate_chat_history=True,
        )
        
        # Warm up robot connection
        try:
            print("🔄 Warming up robot connection...")
            warmup_result = ur5e_tools.get_current_joint_states.invoke({})
            if "❌" not in warmup_result:
                print("✅ Robot connection established")
            else:
                print("⚠️ Robot may not be connected - continuing anyway")
        except Exception as e:
            print(f"⚠️ Warmup failed: {e} - continuing anyway")
        
        print("🤖 UR5e ROSA Agent initialized successfully!")
        
        # Example commands for users
        self.examples = [
            "What are the current joint angles?",
            "Show me the current position of the robot",
            "Move to home position",
            "Move joint 1 by 30 degrees",
            "Move the end effector up by 5 centimeters",
            "Activate the cartesian controller",
            "Emergency stop"
        ]

    @property
    def greeting(self):
        """Welcome message for users"""
        greeting = Text("\n🤖 UR5e AI Agent Ready!\n", style="bold blue")
        greeting.append("I can control your UR5e robot using natural language commands.\n", style="white")
        greeting.append("Try asking me to move the robot, check positions, or switch controllers.\n", style="italic")
        greeting.append(f"\nCommands: 'examples', 'help', 'clear', 'exit'\n", style="dim")
        return greeting

    def choose_example(self):
        """Let user select from example commands"""
        return pyip.inputMenu(
            self.examples,
            prompt="\nSelect an example command:\n",
            numbered=True,
            blank=False,
            timeout=60,
            default="1"
        )

    def get_input(self, prompt: str = "> "):
        """Get user input with prompt"""
        return pyip.inputStr(prompt, default="help")

    async def clear(self):
        """Clear conversation history"""
        self.clear_chat()
        os.system("clear" if os.name != "nt" else "cls")

    def show_help(self, console):
        """Show help information"""
        help_text = Text("UR5e AI Agent Help\n\n", style="bold")
        help_text.append("Available Commands:\n", style="underline")
        help_text.append("• examples - Show example commands\n")
        help_text.append("• clear - Clear conversation history\n")
        help_text.append("• help - Show this help\n")
        help_text.append("• exit - Quit the agent\n\n")
        
        help_text.append("Example Robot Commands:\n", style="underline")
        for i, example in enumerate(self.examples[:4], 1):
            help_text.append(f"{i}. {example}\n")
        
        help_text.append("\nSafety Notes:\n", style="underline red")
        help_text.append("• Robot movements are limited for safety\n")
        help_text.append("• Emergency stop available anytime\n")
        help_text.append("• Always ensure workspace is clear\n")
        
        console.print(Panel(help_text, title="Help", border_style="cyan"))

    async def run_interactive(self):
        """Run the interactive command loop"""
        console = Console()
        
        # Welcome message - show only once at startup
        console.print(Panel(self.greeting, title="UR5e AI Agent", border_style="blue"))
        
        while True:
            try:
                # Just show prompt, not the full greeting each time
                user_input = self.get_input().strip()
                
                if user_input.lower() in ['exit', 'quit']:
                    console.print("👋 Goodbye!", style="bold green")
                    break
                elif user_input.lower() == 'clear':
                    await self.clear()
                    # Show welcome message again after clear
                    console.print(Panel(self.greeting, title="UR5e AI Agent", border_style="blue"))
                    continue
                elif user_input.lower() == 'examples':
                    user_input = self.choose_example()
                elif user_input.lower() == 'help':
                    self.show_help(console)
                    continue
                
                # Process robot command using ROSA's invoke
                console.print(f"\n💭 Processing: {user_input}", style="italic")
                response = self.invoke(user_input)
                
                # Display response
                console.print(Panel(
                    response, 
                    title="🤖 Agent Response", 
                    border_style="green"
                ))
                
                console.print()  # Add spacing
                
            except KeyboardInterrupt:
                console.print("\n👋 Interrupted. Goodbye!", style="bold yellow")
                break
            except Exception as e:
                console.print(f"❌ Error: {e}", style="bold red")

def main():
    """Main entry point"""
    try:
        agent = UR5eROSAAgent(verbose=True, streaming=True)
        asyncio.run(agent.run_interactive())
        
    except KeyboardInterrupt:
        print("\n👋 Agent shutdown requested")
    except Exception as e:
        print(f"❌ Error starting agent: {e}")
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()
