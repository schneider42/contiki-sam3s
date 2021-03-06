/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

package se.sics.cooja.mspmote;

import java.awt.Container;
import java.awt.Image;
import java.awt.MediaTracker;
import java.awt.Toolkit;
import java.io.File;
import java.net.URL;

import javax.swing.Icon;
import javax.swing.ImageIcon;

import org.apache.log4j.Logger;

import se.sics.cooja.AbstractionLevelDescription;
import se.sics.cooja.ClassDescription;
import se.sics.cooja.GUI;
import se.sics.cooja.MoteInterface;
import se.sics.cooja.MoteType;
import se.sics.cooja.Simulation;
import se.sics.cooja.dialogs.CompileContiki;
import se.sics.cooja.dialogs.MessageList;
import se.sics.cooja.dialogs.MessageList.MessageContainer;
import se.sics.cooja.interfaces.IPAddress;
import se.sics.cooja.interfaces.Mote2MoteRelations;
import se.sics.cooja.interfaces.MoteAttributes;
import se.sics.cooja.interfaces.Position;
import se.sics.cooja.interfaces.RimeAddress;
import se.sics.cooja.mspmote.interfaces.MspClock;
import se.sics.cooja.mspmote.interfaces.MspDebugOutput;
import se.sics.cooja.mspmote.interfaces.MspMoteID;
import se.sics.cooja.mspmote.interfaces.SkyByteRadio;
import se.sics.cooja.mspmote.interfaces.TyndallLED;
import se.sics.cooja.mspmote.interfaces.UsciA0Serial;

@ClassDescription("Tyndall")
@AbstractionLevelDescription("Emulated level")
public class TyndallMoteType extends MspMoteType {
  private static Logger logger = Logger.getLogger(TyndallMoteType.class);

  protected MspMote createMote(Simulation simulation) {
    return new TyndallMote(this, simulation);
  }

  public boolean configureAndInit(Container parentContainer, Simulation simulation, boolean visAvailable)
  throws MoteTypeCreationException {

    /* If visualized, show compile dialog and let user configure */
    if (visAvailable) {

      /* Create unique identifier */
      if (getIdentifier() == null) {
        int counter = 0;
        boolean identifierOK = false;
        while (!identifierOK) {
          identifierOK = true;

          counter++;
          setIdentifier("tyndall#" + counter);

          for (MoteType existingMoteType : simulation.getMoteTypes()) {
            if (existingMoteType == this) {
              continue;
            }
            if (existingMoteType.getIdentifier().equals(getIdentifier())) {
              identifierOK = false;
              break;
            }
          }
        }
      }

      /* Create initial description */
      if (getDescription() == null) {
        setDescription("Tyndall Mote Type " + getIdentifier());
      }

      return MspCompileDialog.showDialog(parentContainer, simulation, this, "tyndall");
    }

    /* Not visualized: Compile Contiki immediately */
    if (getIdentifier() == null) {
      throw new MoteTypeCreationException("No identifier");
    }

    final MessageList compilationOutput = new MessageList();

    if (getCompileCommands() != null) {
      /* Handle multiple compilation commands one by one */
      String[] arr = getCompileCommands().split("\n");
      for (String cmd: arr) {
        if (cmd.trim().isEmpty()) {
          continue;
        }

        try {
          CompileContiki.compile(
              cmd,
              null,
              null /* Do not observe output firmware file */,
              getContikiSourceFile().getParentFile(),
              null,
              null,
              compilationOutput,
              true
          );
        } catch (Exception e) {
          MoteTypeCreationException newException =
            new MoteTypeCreationException("Mote type creation failed: " + e.getMessage());
          newException = (MoteTypeCreationException) newException.initCause(e);
          newException.setCompilationOutput(compilationOutput);

          /* Print last 10 compilation errors to console */
          MessageContainer[] messages = compilationOutput.getMessages();
          for (int i=messages.length-10; i < messages.length; i++) {
            if (i < 0) {
              continue;
            }
            logger.fatal(">> " + messages[i]);
          }

          logger.fatal("Compilation error: " + e.getMessage());
          throw newException;
        }
      }
    }

    if (getContikiFirmwareFile() == null ||
        !getContikiFirmwareFile().exists()) {
      throw new MoteTypeCreationException("Contiki firmware file does not exist: " + getContikiFirmwareFile());
    }
    return true;
  }

  public Icon getMoteTypeIcon() {
    Toolkit toolkit = Toolkit.getDefaultToolkit();
    URL imageURL = this.getClass().getClassLoader().getResource("tyndall.png");
    Image image = toolkit.getImage(imageURL);
    MediaTracker tracker = new MediaTracker(GUI.getTopParentContainer());
    tracker.addImage(image, 1);
    try {
      tracker.waitForAll();
    } catch (InterruptedException ex) {
    }
    if (image.getHeight(GUI.getTopParentContainer()) > 0 && image.getWidth(GUI.getTopParentContainer()) > 0) {
      image = image.getScaledInstance((200*image.getWidth(GUI.getTopParentContainer())/image.getHeight(GUI.getTopParentContainer())), 200, Image.SCALE_DEFAULT);
      return new ImageIcon(image);
    }

    return null;
  }

  public Class<? extends MoteInterface>[] getAllMoteInterfaceClasses() {
    return new Class[] {
        Position.class,
        RimeAddress.class,
        IPAddress.class,
        Mote2MoteRelations.class,
        MoteAttributes.class,
        MspClock.class,
        MspMoteID.class,
        SkyByteRadio.class,
        UsciA0Serial.class,
        TyndallLED.class,
        MspDebugOutput.class
    };
  }

  public File getExpectedFirmwareFile(File source) {
    File parentDir = source.getParentFile();
    String sourceNoExtension = source.getName().substring(0, source.getName().length()-2);

    return new File(parentDir, sourceNoExtension + ".tyndall");
  }
  
  protected String getTargetName() {
  	return "tyndall";
  }

}
