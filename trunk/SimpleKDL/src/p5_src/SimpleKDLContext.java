/* ----------------------------------------------------------------------------
 * SimpleKDL
 * ----------------------------------------------------------------------------
 * Copyright (C) 2012 Max Rheiner / Interaction Design Zhdk
 *
 * This file is part of SimpleOpenNI.
 *
 * SimpleOpenNI is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version (subject to the "Classpath" exception
 * as provided in the LICENSE.txt file that accompanied this code).
 *
 * SimpleKDL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleKDL.  If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------------------------
 */
 
package SimpleKDL;

import java.lang.reflect.Method;
import java.net.URL;

import processing.core.*;


public class SimpleKDLContext
{
    final static int SIMPLEOPENNI_VERSION = 10;  // 0.1

    static
    {   // load the nativ shared lib
        String sysStr = System.getProperty("os.name").toLowerCase();
        String libName = "SimpleKDL";
        String archStr = System.getProperty("os.arch").toLowerCase();

        // check which system + architecture
        if(sysStr.indexOf("win") >= 0)
        {   // windows
			String depLib = "orocos-kdl";
			
            if(archStr.indexOf("86") >= 0)
            {   // 32bit
                libName += "32";
                depLib += "32";
			}                
            else if(archStr.indexOf("64") >= 0)
            {
                libName += "64";
                depLib += "64";
			}
			depLib += ".dll";
			   
			String libPath = getLibraryPathWin() + "/SimpleKDL/library/";
			libPath = libPath.replace('/','\\');
			/*
			// doesn't work ??
			// add library path to the system.load       
			System.out.println("java.library.path" + System.getProperty("java.library.path"));
			System.setProperty("java.library.path", 
							   System.getProperty("java.library.path") + ";" + libPath);
			System.out.println("java.library.path" + System.getProperty("java.library.path"));
			*/
			
		    try{
				System.load(libPath + depLib);
			}
			catch(UnsatisfiedLinkError e)
			{
				System.out.println("Can't load SimpleKDL library (" +  libName  + ") : " + e);
			}  
			
         }
         
        else if(sysStr.indexOf("nix") >= 0 || sysStr.indexOf("linux") >=  0 )
        {   // unix
            if(archStr.indexOf("86") >= 0)
                // 32bit
                libName += "32";
            else if(archStr.indexOf("64") >= 0)
                libName += "64";
        }
        else if(sysStr.indexOf("mac") >= 0)
        {     // mac
        }
        
        try{
          System.loadLibrary(libName);
        }
        catch(UnsatisfiedLinkError e)
        {
          System.out.println("Can't load SimpleKDL library (" +  libName  + ") : " + e);
        }        
    }

  public static String getLibraryPathWin() 
  {
    URL url = SimpleKDLContext.class.getResource("SimpleKDLContext.class");
    if (url != null) 
    {
      // Convert URL to string, taking care of spaces represented by the "%20"
      // string.
      String path = url.toString().replace("%20", " ");
      int n0 = path.indexOf('/');

      int n1 = -1;

        n1 = path.indexOf("/SimpleKDL/library"); // location of GSVideo.jar in
                                               // exported apps.

        // In Windows, path string starts with "jar file/C:/..."
        // so the substring up to the first / is removed.
        n0++;


      if ((-1 < n0) && (-1 < n1)) 
        return path.substring(n0, n1);
      else
        return "";
    }
    else
		return "";
   }

    public static void init(PApplet parent)
    {
        parent.println("SimpleKDL Version " + (SIMPLEOPENNI_VERSION / 100) + "." + (SIMPLEOPENNI_VERSION % 100));

        Utils utils = new Utils();
        utils.init(parent);

        Utils.setInst(utils);
    }

}
