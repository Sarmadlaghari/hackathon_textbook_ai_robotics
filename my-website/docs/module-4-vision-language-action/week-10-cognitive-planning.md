---
title: Cognitive Planning - LLMs Translating Natural Language to ROS 2 Actions
sidebar_label: Week 10 - Cognitive Planning with LLMs
sidebar_position: 10
---

# Week 10: Cognitive Planning - LLMs Translating Natural Language to ROS 2 Actions

## Introduction

Welcome to Week 10 of the Vision-Language-Action (VLA) module! This week we'll explore cognitive planning using Large Language Models (LLMs) to translate natural language instructions into executable ROS 2 actions. We'll learn how to leverage the reasoning capabilities of LLMs to create sophisticated robot behaviors that can interpret complex, high-level commands and break them down into specific, executable robot actions.

## Learning Objectives

By the end of this week, you will be able to:
- Understand the role of LLMs in cognitive robotics
- Implement LLM-based natural language understanding for robots
- Design prompt engineering strategies for robotics tasks
- Translate high-level natural language commands into ROS 2 action sequences
- Create robust cognitive planning pipelines that handle ambiguity and errors

## Prerequisites

Before starting this week's content, ensure you have:
- Understanding of ROS 2 fundamentals (Weeks 1-3)
- Experience with voice-to-action systems (Week 9)
- Basic knowledge of natural language processing
- Familiarity with API integration concepts

## 1. Introduction to Cognitive Robotics with LLMs

### 1.1 What is Cognitive Robotics?

Cognitive robotics involves creating robots that can:
- Understand high-level, natural language commands
- Reason about the environment and task requirements
- Plan complex sequences of actions
- Adapt to unexpected situations
- Learn from experience and interaction

### 1.2 Role of LLMs in Cognitive Robotics

Large Language Models enhance cognitive robotics by:
- **Natural Language Understanding**: Interpreting human instructions
- **Reasoning**: Planning multi-step actions
- **Knowledge Integration**: Accessing world knowledge
- **Context Awareness**: Understanding situational context
- **Adaptation**: Learning from interaction

### 1.3 Architecture of LLM-Based Cognitive Systems

- **Input Processing**: Natural language command reception
- **Understanding**: LLM-based semantic analysis
- **Planning**: Action sequence generation
- **Execution**: ROS 2 command execution
- **Feedback**: Result interpretation and learning

## 2. LLM Integration for Robotics

### 2.1 Choosing the Right LLM

Consider these factors for robotics applications:
- **Response Time**: Real-time vs. batch processing
- **Accuracy**: Understanding complex commands
- **Cost**: API usage and computational requirements
- **Privacy**: Handling sensitive data
- **Customization**: Fine-tuning capabilities

### 2.2 Popular LLM Options for Robotics

- **OpenAI GPT**: High capability, good documentation
- **Anthropic Claude**: Strong reasoning, safety focus
- **Google Gemini**: Multimodal capabilities
- **Open Source Models**: Mistral, Llama (for local deployment)
- **Specialized Models**: Fine-tuned for robotics tasks

### 2.3 API Integration Patterns

```python
import openai
import json
from typing import Dict, List, Any

class LLMRobotPlanner:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        openai.api_key = api_key
        self.model = model
        self.system_prompt = self._build_system_prompt()

    def _build_system_prompt(self) -> str:
        return """
        You are a robotic task planner. Your job is to interpret natural language commands
        and translate them into structured robot actions for a ROS 2 system.

        Available actions:
        - move_to(location): Move robot to specified location
        - pick_object(object_name, location): Pick up an object
        - place_object(object_name, location): Place an object at location
        - navigate_to(location): Navigate to location
        - detect_object(object_type): Detect objects of specified type
        - wait(duration): Wait for specified duration
        - report_status(): Report current robot status

        Respond with a JSON object containing:
        {
            "action_sequence": [
                {
                    "action": "action_name",
                    "parameters": {"param1": "value1", ...}
                }
            ],
            "reasoning": "Brief explanation of the plan"
        }

        Be specific about locations and objects. If information is ambiguous,
        ask for clarification.
        """

    def plan_task(self, command: str) -> Dict[str, Any]:
        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.1  # Low temperature for consistent responses
        )

        try:
            result = json.loads(response.choices[0].message['content'])
            return result
        except json.JSONDecodeError:
            # Handle cases where response isn't valid JSON
            return {"action_sequence": [], "reasoning": "Failed to parse response"}
```

## 3. Prompt Engineering for Robotics

### 3.1 System Prompt Design

Effective system prompts for robotics should include:
- **Role Definition**: Clearly define the LLM's role
- **Action Vocabulary**: List available robot actions
- **Format Requirements**: Specify output format
- **Context Information**: Provide relevant environment info
- **Safety Guidelines**: Include safety constraints

### 3.2 Few-Shot Learning Examples

```python
def get_few_shot_examples() -> List[Dict[str, str]]:
    return [
        {
            "role": "user",
            "content": "Go to the kitchen and bring me a cup of coffee."
        },
        {
            "role": "assistant",
            "content": json.dumps({
                "action_sequence": [
                    {"action": "navigate_to", "parameters": {"location": "kitchen"}},
                    {"action": "detect_object", "parameters": {"object_type": "cup"}},
                    {"action": "pick_object", "parameters": {"object_name": "cup", "location": "kitchen counter"}},
                    {"action": "navigate_to", "parameters": {"location": "coffee machine"}},
                    {"action": "place_object", "parameters": {"object_name": "cup", "location": "coffee machine tray"}},
                    {"action": "navigate_to", "parameters": {"location": "your location"}}
                ],
                "reasoning": "First navigate to kitchen, detect cup, pick it up, then go to coffee machine to place cup, then return."
            })
        },
        {
            "role": "user",
            "content": "Clean the table in the living room."
        },
        {
            "role": "assistant",
            "content": json.dumps({
                "action_sequence": [
                    {"action": "navigate_to", "parameters": {"location": "living room"}},
                    {"action": "detect_object", "parameters": {"object_type": "debris"}},
                    {"action": "pick_object", "parameters": {"object_name": "debris", "location": "living room table"}},
                    {"action": "navigate_to", "parameters": {"location": "trash bin"}},
                    {"action": "place_object", "parameters": {"object_name": "debris", "location": "trash bin"}},
                    {"action": "report_status", "parameters": {}}
                ],
                "reasoning": "Navigate to living room, detect debris on table, pick up debris, dispose in trash bin, report completion."
            })
        }
    ]
```

### 3.3 Context-Aware Prompting

Include environmental context in prompts:
- Current robot location
- Available objects and their positions
- Recent actions and results
- User preferences and history

## 4. Action Planning and Execution

### 4.1 Action Representation

Standardize action representations:
- **Action Name**: String identifier for the action
- **Parameters**: Dictionary of required parameters
- **Preconditions**: Conditions that must be met
- **Effects**: Expected outcomes
- **Duration**: Estimated execution time

### 4.2 Plan Validation

Validate plans before execution:
- Check action availability
- Verify parameter validity
- Ensure preconditions are met
- Detect potential conflicts

### 4.3 ROS 2 Action Integration

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')

        # Initialize LLM planner
        self.llm_planner = LLMRobotPlanner(api_key="your-api-key")

        # ROS 2 publishers and action clients
        self.move_client = ActionClient(self, MoveBaseAction, 'move_base')
        self.command_pub = self.create_publisher(String, 'robot_commands', 10)

        # Subscribe to natural language commands
        self.command_sub = self.create_subscription(
            String, 'natural_language_commands', self.command_callback, 10)

    def command_callback(self, msg):
        # Plan task using LLM
        plan_result = self.llm_planner.plan_task(msg.data)

        # Execute the plan
        self.execute_plan(plan_result)

    def execute_plan(self, plan_result):
        action_sequence = plan_result.get('action_sequence', [])

        for action in action_sequence:
            action_name = action['action']
            parameters = action['parameters']

            if action_name == 'navigate_to':
                self.execute_navigate_to(parameters['location'])
            elif action_name == 'pick_object':
                self.execute_pick_object(
                    parameters['object_name'],
                    parameters['location']
                )
            elif action_name == 'place_object':
                self.execute_place_object(
                    parameters['object_name'],
                    parameters['location']
                )
            # Add other action handlers as needed

    def execute_navigate_to(self, location):
        # Convert location to coordinates and navigate
        goal = MoveBaseGoal()
        # Set goal coordinates based on location name
        self.move_client.send_goal(goal)

    def execute_pick_object(self, object_name, location):
        # Implementation for picking object
        pass

    def execute_place_object(self, object_name, location):
        # Implementation for placing object
        pass
```

## 5. Handling Ambiguity and Errors

### 5.1 Ambiguity Detection

Identify when LLM responses are ambiguous:
- Missing parameters
- Unclear locations
- Conflicting actions
- Unavailable actions

### 5.2 Clarification Strategies

Implement clarification mechanisms:
- Ask for missing information
- Present options for ambiguous choices
- Confirm interpretations before execution
- Use context to resolve ambiguity

```python
class AmbiguityResolver:
    def __init__(self):
        self.known_locations = {
            "kitchen": {"x": 1.0, "y": 2.0},
            "living room": {"x": 3.0, "y": 1.0},
            "bedroom": {"x": 0.5, "y": 4.0}
        }

    def resolve_ambiguity(self, plan_result, environment_context):
        action_sequence = plan_result.get('action_sequence', [])
        resolved_actions = []

        for action in action_sequence:
            if self._has_ambiguity(action):
                resolved_action = self._clarify_action(action, environment_context)
                resolved_actions.append(resolved_action)
            else:
                resolved_actions.append(action)

        plan_result['action_sequence'] = resolved_actions
        return plan_result

    def _has_ambiguity(self, action):
        # Check for missing or unclear parameters
        if action['action'] == 'navigate_to':
            location = action['parameters'].get('location', '').lower()
            if location not in self.known_locations:
                return True
        return False

    def _clarify_action(self, action, context):
        # Implement clarification logic
        # This might involve asking user for clarification
        return action  # Placeholder
```

### 5.3 Error Recovery

Implement error handling and recovery:
- Monitor execution for failures
- Retry failed actions
- Generate alternative plans
- Report errors to users

## 6. Safety and Validation

### 6.1 Safety Constraints

Implement safety checks:
- Physical safety limits
- Environmental constraints
- User safety requirements
- Robot capability limits

### 6.2 Plan Verification

Verify plans meet safety requirements:
- Check for dangerous actions
- Validate environmental feasibility
- Ensure robot can execute planned actions
- Confirm safety constraints are met

### 6.3 Human-in-the-Loop

Include human oversight:
- Plan approval before execution
- Real-time monitoring
- Emergency stop capabilities
- Manual override options

## 7. Performance Optimization

### 7.1 Caching Strategies

Improve performance with caching:
- Cache common command interpretations
- Store frequently used plans
- Cache environmental information
- Cache LLM responses when appropriate

### 7.2 Local Processing

Consider local processing options:
- Run smaller models locally
- Cache models in memory
- Optimize API usage
- Use edge computing for real-time tasks

### 7.3 Asynchronous Processing

Implement asynchronous processing:
- Process commands in background
- Execute actions while planning next steps
- Handle multiple commands concurrently
- Provide feedback during execution

## 8. Advanced Cognitive Planning

### 8.1 Multi-Modal Integration

Combine with other sensors:
- Vision systems for object recognition
- Audio systems for voice commands
- Tactile sensors for manipulation
- Environmental sensors for context

### 8.2 Learning and Adaptation

Implement learning capabilities:
- Learn from successful executions
- Adapt to user preferences
- Improve command understanding
- Optimize plan efficiency

### 8.3 Collaborative Planning

Enable multi-robot coordination:
- Coordinate actions between multiple robots
- Share environmental information
- Handle complex multi-robot tasks
- Manage resource allocation

## 9. Practical Implementation

### 9.1 Complete Cognitive Planning System

```python
import rclpy
import openai
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from typing import Dict, Any, List

class CompleteCognitivePlanner(Node):
    def __init__(self):
        super().__init__('complete_cognitive_planner')

        # Initialize components
        self.llm_client = openai.OpenAI(api_key="your-api-key")
        self.ambiguity_resolver = AmbiguityResolver()

        # ROS 2 interfaces
        self.command_sub = self.create_subscription(
            String, 'natural_language_commands', self.process_command, 10)
        self.action_pub = self.create_publisher(String, 'robot_actions', 10)

        # Context tracking
        self.robot_location = "base_station"
        self.environment_objects = {}

        self.get_logger().info("Cognitive Planner initialized")

    def process_command(self, msg):
        try:
            # Get environmental context
            context = self._get_environment_context()

            # Generate plan with LLM
            plan = self._generate_plan(msg.data, context)

            # Resolve ambiguities
            resolved_plan = self.ambiguity_resolver.resolve_ambiguity(plan, context)

            # Validate plan safety
            if self._validate_plan_safety(resolved_plan):
                # Execute plan
                self._execute_plan(resolved_plan)
            else:
                self.get_logger().error("Plan failed safety validation")

        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")

    def _get_environment_context(self) -> Dict[str, Any]:
        return {
            "robot_location": self.robot_location,
            "available_objects": self.environment_objects,
            "time_of_day": "day",  # Could come from system
            "user_preferences": {}  # Could be loaded from user profile
        }

    def _generate_plan(self, command: str, context: Dict[str, Any]) -> Dict[str, Any]:
        system_prompt = f"""
        You are a cognitive robot planner. Plan robot actions based on user commands.
        Current context: {json.dumps(context)}

        Available actions: navigate_to, pick_object, place_object, detect_object, wait, report_status
        Respond in JSON format with action_sequence and reasoning.
        """

        response = self.llm_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.1
        )

        return json.loads(response.choices[0].message.content)

    def _validate_plan_safety(self, plan: Dict[str, Any]) -> bool:
        # Implement safety validation logic
        return True  # Placeholder

    def _execute_plan(self, plan: Dict[str, Any]):
        action_sequence = plan.get('action_sequence', [])
        for action in action_sequence:
            self.execute_single_action(action)

    def execute_single_action(self, action: Dict[str, Any]):
        # Publish action to appropriate ROS 2 interface
        action_msg = String()
        action_msg.data = json.dumps(action)
        self.action_pub.publish(action_msg)
```

## 10. Testing and Evaluation

### 10.1 Plan Quality Metrics

Evaluate cognitive planning systems:
- **Success Rate**: Percentage of successfully executed commands
- **Planning Time**: Time to generate action plans
- **Accuracy**: Correctness of action interpretation
- **Robustness**: Handling of ambiguous or complex commands

### 10.2 Human-Robot Interaction Metrics

Measure interaction quality:
- **User Satisfaction**: Subjective evaluation
- **Task Completion**: Successful task execution
- **Error Recovery**: Handling of failures
- **Naturalness**: How natural the interaction feels

## Exercises

1. **Basic Integration**: Implement a simple LLM-based command interpreter
2. **Prompt Engineering**: Design effective prompts for specific robot tasks
3. **Plan Validation**: Add safety checks to your cognitive planning system
4. **Multi-step Tasks**: Create complex task planning with error handling

## Summary

This week we explored cognitive planning using LLMs to translate natural language into ROS 2 actions. We learned about prompt engineering, action planning, ambiguity resolution, and safety considerations. Cognitive planning enables robots to understand complex, high-level commands and execute sophisticated behaviors that adapt to their environment and user needs.

## References

- [OpenAI API Documentation](https://platform.openai.com/docs/)
- [ROS 2 Navigation Documentation](https://navigation.ros.org/)
- [Large Language Models for Robotics](https://arxiv.org/abs/2303.17015)
- [Cognitive Robotics Research](https://ieeexplore.ieee.org/document/9146621)