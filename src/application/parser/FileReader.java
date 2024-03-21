package application.parser;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class FileReader{
    public static List<String> readFile(String filename) throws FileNotFoundException {
		try {
			List<String> output = new ArrayList<String>();
			File file = new File(filename);
			Scanner reader = new Scanner(file);
			while (reader.hasNextLine()) 
				output.add(reader.nextLine());
			reader.close();
			return output;
	    } catch (FileNotFoundException e) {
	    	throw e;
	    }
	}
    
    public static String findUniqueFolder(String folder, String from) throws IOException {
    	File dir = new File(from);
    	List<String> path = findUniqueFolderRecursive(folder, dir);
    	if(path == null) return null;
    	String pathString = "";
    	for(int i = path.size()-1;i >= 0;i--) {
    		pathString += path.get(i);
    		if(i != 0) pathString += "/";
    	}
		return pathString;
    }
    
    private static List<String> findUniqueFolderRecursive(String folder, File dir) throws IOException {
    	File[] filesList = dir.listFiles();
    	if(filesList.length == 0) return null;
		for (File file : filesList) {
			if(!file.isDirectory()) continue;
			if(file.getName().equals(folder)) {
				List<String> path = new ArrayList<String>();
				path.add(file.getName());
				path.add(dir.getName());
				return path;
			}
			List<String> pathList = findUniqueFolderRecursive(folder, file);
			if(pathList == null) continue;
			pathList.add(dir.getName());
			return pathList;
		}
		return null;
    }
}




















