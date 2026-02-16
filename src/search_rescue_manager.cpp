#include <cstdlib>
#include <iostream>
#include <sys/wait.h>

int main(int, char**) {
	int ret = std::system("rosrun assignment_3 search_rescue_search");

	if (ret < 0) {
		std::cerr << "failed to invoke rosrun\n";
		return 1;
	}

	int exit_code = WEXITSTATUS(ret);

	if (exit_code == 0){
		std::cout << "search_rescue_search finished successfully!\n";

		int ret = std::system("rosrun assignment_3 search_rescue_rescue");

		if (ret < 0) {
			std::cerr << "failed to invoke rosrun\n";
			return 1;
		}
		
		int exit_code = WEXITSTATUS(ret);
		if (exit_code == 0){
			std::cout << "search_rescue_rescue finished successfully!\n";
		}
		else{
			std::cout << "search_rescue_rescue exited with code " << exit_code << "\n";
		}

	}
	else{
		std::cout << "search_rescue_search exited with code " << exit_code << "\n";
	}
	return exit_code;
}
