This project is an A level computer science NEA for the OCR board. The project is:

An air traffic resolver which takes aircraft, and vectors them for landing. It handles any losses of separation and conflicts, and should schedule aircraft for landing appropriately. It should use as little holding and delays for aircraft as possible.

When developing code ALWAYS consider the best practices, such as where to place the code, and the DRY principle (DO NOT REPEAT YOURSELF)

This project relies on the separation of the frontend from the backend. They should be separate, such that if desired, it would work headlessly. 

Keep all code clean and concise, using the most understandable structures, and always plan the step ahead before implementing it. Try to keep all logic out of header files if possible, where appropriate, unless it is not feasible without making the code messy

Remember, this is an A level project, not a univeristy project. Code can be much simpler than you expect. Keep it small and simple

You should add logging when sensible, using the correct kind of log. Logging is provided from src/common/logger.h and logger.cpp where a full system is set up already