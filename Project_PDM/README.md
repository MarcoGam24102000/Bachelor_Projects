
# Athletics Management App

## Project Overview

This project is an athletics management application designed to streamline the management of various panels including athlete, coach, physiotherapist, and clubs. Additionally, it features a history panel and more to provide comprehensive support for managing athletic activities.

## Features

- **Athlete Panel**: Manage athlete profiles, track performance, and monitor progress.
- **Coach Panel**: Assign coaches to athletes, manage training schedules, and track coaching activities.
- **Physiotherapist Panel**: Record and manage physiotherapy sessions and monitor athlete recovery.
- **Clubs Panel**: Manage club details, memberships, and related activities.
- **History Panel**: Track the history of various activities and interactions within the app.
- **Bluetooth Integration**: Utilize Bluetooth services for enhanced connectivity and data transfer.

## Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/yourusername/athletics-management-app.git
   cd athletics-management-app
   ```

2. **Install Dependencies**
   - Ensure you have the required tools and dependencies installed, such as Java, Android SDK, etc.

3. **Build the Project**
   - Use your preferred IDE (e.g., Android Studio) to build the project.

## Usage

1. **Launching the App**
   - Open the project in your IDE and run the app on an emulator or a physical device.

2. **Navigating the App**
   - Use the main navigation to access different panels (Athlete, Coach, Physiotherapist, Clubs, History, etc.).

## Code Structure

### Activities

- **AddActivity**
  - Manages the addition of activities.
- **AddCouch**
  - Manages the addition of coaches.
- **AddFisio**
  - Manages the addition of physiotherapists.
- **CalendarActivity**
  - Manages calendar-related activities.
- **DeviceControlActivity**
  - Controls and manages connected devices via Bluetooth.
- **DeviceScanActivity**
  - Scans for Bluetooth devices.
- **ExampleInstrumentedTest**
  - Contains instrumented tests which run on an Android device.
- **ExampleUnitTest**
  - Contains unit tests which run on the development machine (host).
- **FirstActivity**
  - The first activity that users interact with.
- **History**
  - Manages the history of activities.
- **MainActivity**
  - The main entry point of the app, managing overall navigation and main features.
- **MainActivityCouch**
  - Manages the main functionalities related to coaches.
- **MainActivityEnter**
  - Entry activity for the app.
- **MainActivityFisio**
  - Manages the main functionalities related to physiotherapists.
- **MainActivitySensor**
  - Handles sensor-related activities and data.
- **MapsActivity**
  - Manages map-related functionalities.
- **NextTraining**
  - Manages upcoming training sessions.
- **PermissionUtils**
  - Contains utility methods for handling permissions.
- **SearchCouch**
  - Manages the search functionality for coaches.
- **SearchFisio**
  - Manages the search functionality for physiotherapists.
- **SendTraining**
  - Manages the functionality to send training data.
- **ShowAthletesCouch**
  - Displays athletes managed by a coach.
- **ShowAthletesFisio**
  - Displays athletes managed by a physiotherapist.
- **SpecialityCouch**
  - Manages specialities of coaches.
- **SplashScreenActivity**
  - Manages the splash screen.
- **TrainingsWeekActivity**
  - Manages weekly training activities.
- **UpdateActivity**
  - Manages updates to activities and profiles.

## Contributing

1. **Fork the Repository**
2. **Create a New Branch**
   ```bash
   git checkout -b feature-branch
   ```
3. **Make Your Changes**
4. **Commit Your Changes**
   ```bash
   git commit -m "Add some feature"
   ```
5. **Push to the Branch**
   ```bash
   git push origin feature-branch
   ```
6. **Create a Pull Request**

