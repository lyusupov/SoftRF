# A GitHub Manifesto
### Notes on contributing to my repositories
Jack Christensen  
Jan 2018

Surely Git and GitHub are wonderful tools. They make coding and collaboration so much easier. I'm equally impressed with the open source movement, and with the Arduino ecosystem in particular.

I'm just one guy, mostly a hobbyist. Posting my projects to GitHub is my way of giving back a little to the community. It's very gratifying that some of my code has received a modicum of popularity.

Like many things, this has been somewhat of a double-edged sword. Especially since I tend to be a pretty busy guy with many varied interests.

First, I am always interested in bug reports. Please raise an issue in the appropriate repository and please please please include a good, concise description of the issue and a Short, Self Contained, Correct (Compilable), Example (see [sscce.org](http://www.sscce.org/)). I will need to be able to reproduce the issue, with minimal hardware, and without installing a dozen other libraries. I work exclusively with the AVR architecture so most times I will not be able to reproduce issues on other platforms. (There have been one or two occasions where relatively simple changes have been made to accommodate another platform; I am not necessarily averse to these.)

Second, bug reports should always be for problems with *my* code. I will not use GitHub to help you with *your* code, even if you happen to be using one of my libraries. Please use the [Arduino forum](https://forum.arduino.cc/) or other such venue instead.

Finally, pull requests can be problematic, especially if they represent enhancements rather than fixes. I seldom intend my code to be all things to all people. This is mostly a hobby activity and I have very limited bandwidth. Reviewing and managing PRs requires time that I do not often have. Sometimes a PR will take a library in a direction that I'm not interested in. Sometimes a PR will be counter to my original design intent. No doubt the author of a PR thinks that their new feature is the best thing since canned beer, but if I don't happen to share that opinion, then I'll decline it. OTOH, I am certainly capable of making stupid mistakes and missing absolutely fundamental things, and I do appreciate it when these are pointed out.

All this to say, that if I do decline a request, please do not take it personally. Feel free to consider it my problem, not yours. At the end of the day, it's my code, and I reserve the right to decline issues or PRs for any reason, or for no reason at all. But here is the beauty of open source. You can always fork the repository and have your way with it.
