# Bottle Classifier & Cocktail Resolver
An application for detecting and classifying bottles, as well as displaying appropriate cocktail recipes based on the classified bottles.

Achieved using trained HAAR/LBP cascades for detection and colour histograms generated from detected ROI to improve classification performance. Currently classifies 'Smirnoff Vodka' and 'Jack Daniels' bottles, as well as 'Red Bull' cans, 'Monster Energy' cans and an orange juice carton.

Example output can be seen in "results/".

## Usage
### Main Application
Ensure MySQL server is running and that the database has been created (import latest: "database/backup/bottle_classifier_backup_x.sql"). Once target bottles have been classified, press the space key to resolve cocktail recipes.
```
cd classifier
python3 perform_cascade.py -h
python3 perform_cascade.py
python3 perform_cascade.py -i ../test_data/test_multi.jpg
```

### Recipe Scraper
Scrap a singular recipe or multiple recipes by ingredient from "https://cocktail.uk.com". Add "-s" flag to save to MySQL database. 
```
cd scraper
python3 recipe_scraper.py -u https://www.cocktail.uk.com/cocktails/containing/vodka
python3 recipe_scraper.py -u https://www.cocktail.uk.com/cocktails/cosmopolitan
```

### Generate/Flush Histograms
After adding new images to "histogram/histogram_data", ".histo" histogram files can be generated. These must be entered into the database manually after creation.
```
cd histogram
./refresh_histograms.sh
python3 create_histogram.py -i histogram_data/oj.jpg
```

## Dependencies
+ MySQL database
+ Python 3
+ OpenCV 3.1.0 & NumPy
+ PyMySQL & peewee
+ Beautiful Soup 4
+ Flask
+ argparse
+ webbrowser
+ pickle
