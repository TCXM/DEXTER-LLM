# 🌐 Task Generation LLM

Task Generation LLM is a powerful tool designed to assist in generating tasks for various applications using large language models. It leverages state-of-the-art natural language processing techniques to provide accurate and efficient task generation.

## ⬇️ Installation

This repo is a ROS package, so you can directly clone it to your own workspace. While the **recommended** usage is adding it as a git submodule, which can keep up with the newest version.

1. Create your workspace:
    ```bash
	mkdir -p your_ws/src
	cd your_ws
	catkin_make
	git init
	```
2. Add this repository as submodule:
	```bash
	git submodule add https://github.com/TCXM/task_gen_llm.git src/task_gen_llm
	```

## 🧭 Input and Output Format

### Input Format:
```json
{
	"Events and Feedback": [
		"We have detected flame_1 at (2, 2, 2)",
		"..."
	]
}
```
### Output Format:

Output format is defined in prompt template.