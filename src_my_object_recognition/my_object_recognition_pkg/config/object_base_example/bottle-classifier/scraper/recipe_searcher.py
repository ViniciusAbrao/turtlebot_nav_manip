import os, sys
from decimal import Decimal
from flask import Flask, render_template, Response, request, jsonify
import webbrowser
from collections import OrderedDict

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/database")
from db_models import *
from db import *

# get list of recipes by bottles supplied
def get_recipes(bottle_ids):
	bottles_count = len(bottle_ids)
	ingredients_str = ""
	bottles_detected = []
	detected_ingredient_names = []
	
	# construct comma separated recipe ids and trim last comma
	for bottle_id, bottle in bottle_ids.items():
		ingredient_id = get_ingredient_id(bottle_id)
		ingredients_str += str(ingredient_id) + ", "
		detected_ingredient_names.append(get_ingredient_name(ingredient_id))
	
	ingredients_str = ingredients_str[:-2]
	
	# join recipes and ingredients tables
	recipes = get_recipes_by_ingredients(ingredients_str)
	
	# group recipes with their number of ingredients returned
	results_grouping = OrderedDict()
	
	for recipe in recipes.execute():
		if recipe.recipe_id in results_grouping:
			results_grouping[recipe.recipe_id] += 1
		else:
			results_grouping[recipe.recipe_id] = 1
	
	# store recipe/ingredient info to be passed onto server for display
	final_recipes = []
	counter = 0
	
	for recipe_id, count in results_grouping.items():
		if count == bottles_count:
			# get recipe details
			recipe_details = Recipe.select().where(Recipe.recipe_id == recipe_id).get()
			# convert to sentence case
			recipe_details.instructions = '. '.join(i.capitalize() for i in recipe_details.instructions.split('. '))
			recipe_details.name = ' '.join([word.capitalize() for word in recipe_details.name.split(' ')])
			# add recipe
			final_recipes.append([recipe_details, []])
			
			ingredients = get_ingredients_by_recipe(recipe_id)

			for ingredient in ingredients.execute():
				#print("  " + ingredient.amount + " " + ingredient.name)
				final_recipes[counter][1].append({"amount" : ingredient.amount, "name" : ingredient.name.strip()})
			
			counter += 1
			
	# format ingredient names for display
	ingredient_names_str = ""
	for i, name in enumerate(detected_ingredient_names):
		ingredient_names_str += name
		if (i < len(detected_ingredient_names) - 2):
			ingredient_names_str += ", "
		elif (i == len(detected_ingredient_names) - 2):
			ingredient_names_str += " and "
		
	ingredient_names = {"names" : detected_ingredient_names, "names_str" : ingredient_names_str}
	
	# start server
	launch_server(final_recipes, ingredient_names)


# launch server for recipe viewing
def launch_server(recipes, ingredient_names):
	try:	
		app = Flask(__name__)
		app.config['TEMPLATES_AUTO_RELOAD'] = True

		@app.route('/')
		def index():
			# serve home page
			return render_template('index.html', recipes=recipes, ingredient_names=ingredient_names)
		
		# open browser
		webbrowser.open("http://127.0.0.1:8080/", new=2)
		# start server
		app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
	
	except KeyboardInterrupt:
		print("> Server closed")
	


