
Set up the run configurations in Android Studio:

Go to Run → Edit Configurations → + → Gradle, enter the task deploySloth, and save. 

Android Studio won't autocomplete this task name, but typing it in works.

Edit the normal TeamCode configuration. 

 "Before launch," add a Gradle task removeSlothRemote, 

with :TeamCode as the Gradle project, and move it to the top. This clears the hot-loaded code whenever you do a full install, so the two don't conflict.

After that, use deploySloth for everyday changes. It usually takes under a second.


