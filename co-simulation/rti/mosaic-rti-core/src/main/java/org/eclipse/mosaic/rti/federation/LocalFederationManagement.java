/*
 * Copyright (c) 2020 Fraunhofer FOKUS and others. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contact: mosaic@fokus.fraunhofer.de
 */

package org.eclipse.mosaic.rti.federation;

import org.eclipse.mosaic.lib.util.ProcessLoggingThread;
import org.eclipse.mosaic.rti.api.ComponentProvider;
import org.eclipse.mosaic.rti.api.FederateAmbassador;
import org.eclipse.mosaic.rti.api.FederateExecutor;
import org.eclipse.mosaic.rti.api.FederationManagement;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.WatchDog;
import org.eclipse.mosaic.rti.api.parameters.FederateDescriptor;

import ch.qos.logback.classic.LoggerContext;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Collection;
import java.util.HashMap;

/**
 * This implementation of <code>FederationManagement</code> allows local
 * administration only.
 *
 * @see FederationManagement
 */
public class LocalFederationManagement implements FederationManagement {

    /**
     * Constant representing the local host address.
     */
    protected static final String LOCALHOST = "localhost";

    protected final Logger log;

    /**
     * The simulation components of this federation.
     */
    protected final ComponentProvider federation;

    /**
     * Mapping between federation id and federation descriptors.
     */
    protected final HashMap<String, FederateDescriptor> federateDescriptors = new HashMap<>();

    /**
     * Mapping between federation id and federation ambassador instances.
     */
    protected final HashMap<String, FederateAmbassador> federateAmbassadors = new HashMap<>();

    protected WatchDog watchDog;

    /**
     * Constructor with string identifying a federation as parameter.
     *
     * @param federation federation
     */
    public LocalFederationManagement(ComponentProvider federation) {
        this.log = LoggerFactory.getLogger(FederationManagement.class);
        this.federation = federation;
    }

    @Override
    public void createFederation() {
        this.log.info("Start federation with id '{}'", federation.getFederationId());
    }

    @Override
    public void addFederate(FederateDescriptor descriptor) throws Exception {
        this.log.info("Add ambassador/federate with id '{}'", descriptor.getId());
        if (descriptor.isToDeployAndUndeploy()) {
            this.deployFederate(descriptor);
        }

        if (descriptor.isToStartAndStop()) {
            this.startFederate(descriptor);
        }

        descriptor.getAmbassador().setRtiAmbassador(federation.createRtiAmbassador(descriptor.getId()));
        this.federateDescriptors.put(descriptor.getId(), descriptor);
        this.federateAmbassadors.put(descriptor.getId(), descriptor.getAmbassador());
    }

    @Override
    public boolean isFederateJoined(String federateId) {
        return this.federateDescriptors.containsKey(federateId);
    }

    @Override
    public FederateAmbassador getAmbassador(String federateId) {
        return this.federateAmbassadors.get(federateId);
    }

    @Override
    public Collection<FederateAmbassador> getAmbassadors() {
        return this.federateAmbassadors.values();
    }

    @Override
    public void stopFederation() throws Exception {
        for (FederateDescriptor handle : this.federateDescriptors.values()) {

            if (handle.isToStartAndStop()) {
                this.stopFederate(handle, true);
            }

            if (handle.isToDeployAndUndeploy() && handle.getHost() != null) {
                this.undeployFederate(handle);
            }
        }
        this.federateDescriptors.clear();
    }

    private void copyWithAttributes(final Path sourcePath, final Path targetPath) throws IOException {
        Files.createDirectories(targetPath);
        Files.walkFileTree(sourcePath, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult preVisitDirectory(
                    final Path dir,
                    final BasicFileAttributes attrs
            ) throws IOException {
                Files.createDirectories(
                        targetPath.resolve(sourcePath.relativize(dir))
                );
                return FileVisitResult.CONTINUE;
            }

            @Override
            public FileVisitResult visitFile(
                    final Path file,
                    final BasicFileAttributes attrs
            ) throws IOException {
                Files.copy(
                        file,
                        targetPath.resolve(sourcePath.relativize(file)),
                        StandardCopyOption.COPY_ATTRIBUTES,
                        StandardCopyOption.REPLACE_EXISTING,
                        LinkOption.NOFOLLOW_LINKS
                );
                return FileVisitResult.CONTINUE;
            }
        });
    }

    /**
     * Deploys a federate represented by its descriptor on the local machine.
     *
     * @param descriptor federate descriptor consisting all necessary data to deploy a
     *                   federate
     * @throws Exception if the federate could not be deployed
     */
    protected void deployFederate(FederateDescriptor descriptor) throws Exception {
        File hostDeployDir = new File(descriptor.getHost().workingDirectory);

        if (!hostDeployDir.exists()) {
            log.debug("Destination directory does not exist, try to create it.");
            if (!hostDeployDir.mkdirs()) {
                log.warn("Could not create directory {}", hostDeployDir.toString());
            }
        }

        if (hostDeployDir.isDirectory()) {
            final File fedDeployDir = new File(hostDeployDir, descriptor.getId());
            log.info("Deploying federate '{}' locally in {}", descriptor.getId(), fedDeployDir);

            if (fedDeployDir.isDirectory()) {
                removeDirectory(fedDeployDir);
                if (!fedDeployDir.mkdirs()) {
                    log.warn("Could not create directory {}", fedDeployDir.toString());
                }
            }

            if (descriptor.getBinariesDir() != null && descriptor.getBinariesDir().exists()) {
                final Path sourcePath = descriptor.getBinariesDir().toPath();
                final Path targetPath = fedDeployDir.toPath();
                if (sourcePath.toFile().exists()) {
                    copyWithAttributes(sourcePath, targetPath);
                }
            }

            if (descriptor.getConfigDir() != null) {
                final Path sourcePath = descriptor.getConfigDir().toPath();
                final Path targetPath;
                if (descriptor.getConfigTargetPath() != null) {
                    targetPath = fedDeployDir.toPath().resolve(descriptor.getConfigTargetPath());
                } else {
                    targetPath = fedDeployDir.toPath();
                }

                if (sourcePath.toFile().exists()) {
                    copyWithAttributes(sourcePath, targetPath);
                } else {
                    log.warn("There are no configuration files in {} to copy.", sourcePath);
                }
            }
        } else {
            throw new IllegalValueException(hostDeployDir + " (working directory) is not a directory.");
        }
        this.log.debug("finished deploying {}", descriptor.getId());
    }

    /**
     * Starts a federate represented by its handle on the local machine.
     *
     * @param handle federate handle consisting all necessary data to start a
     *               federate
     * @throws Exception if the federate could not be started
     */
    protected void startFederate(FederateDescriptor handle) throws Exception {

        File dir = new File(handle.getHost().workingDirectory, handle.getId());

        final FederateExecutor federateExecutor = handle.getFederateExecutor();

        this.log.info("Starting federate '{}' locally in {}", handle.getId(), dir);
        this.log.debug(" - Federate executor: {}", federateExecutor.toString());

        final Process p = federateExecutor.startLocalFederate(dir);
        if (p == null) {
            return;
        }

        //make the process known to the watchdog(and thus ensuring its termination)
        if (watchDog != null) {
            watchDog.attachProcess(p);
        }

        // determine the ambassadors name by it's class
        String ambassadorName = handle.getAmbassador().getClass().getName();

        // read error output of process in an extra thread
        ProcessLoggingThread errorLoggingThread = new ProcessLoggingThread(
                LoggerFactory.getLogger(ambassadorName + "Error"),
                p.getErrorStream(),
                ambassadorName,
                ProcessLoggingThread.Level.Error
        );
        errorLoggingThread.start();

        // call connectToFederateMethod of the current federate an extract
        // possible output from the federates' output stream (e.g. port number...)
        // note: error- and input streams were read in this class now due to conflicting stream access
        handle.getAmbassador().connectToFederate(LOCALHOST, p.getInputStream(), p.getErrorStream());

        // read the federates stdout in an extra thread and add this to our logging instance
        ProcessLoggingThread outputLoggingThread = new ProcessLoggingThread(
                LoggerFactory.getLogger(ambassadorName + "Output"),
                p.getInputStream(),
                ambassadorName,
                ProcessLoggingThread.Level.Info
        );
        outputLoggingThread.start();
    }

    /**
     * Stops a federate represented by its handle on the local machine.
     *
     * @param handle federate handle consisting all necessary data to stop a federate
     * @throws Exception if the federate could not be stopped
     */
    protected void stopFederate(FederateDescriptor handle, boolean forceStop) throws Exception {
        if (handle.getFederateExecutor() != null) {
            handle.getFederateExecutor().stopLocalFederate();
        }
    }

    /**
     * Undeploys a federate represented by its handle on the local machine.
     *
     * @param handle federate handle consisting all necessary data to undeploy a
     *               federate
     * @throws Exception if the federate could not be undeployed
     */
    protected void undeployFederate(FederateDescriptor handle) throws Exception {
        final File execDir = new File(handle.getHost().workingDirectory, handle.getId());

        this.copyFromFederateLogDir(handle, execDir);
        this.removeDirectory(execDir);
    }

    /**
     * Copies files from the federate execution dir to the actual logging-directory (e.g. SUMO output files)
     */
    private void copyFromFederateLogDir(FederateDescriptor handle, File executionDir) throws IOException {
        final File federateLoggingDir = new File(executionDir, "log");
        if (executionDir.exists() && federateLoggingDir.exists()) {
            final Path sourcePath = federateLoggingDir.toPath();
            final File targetDir = new File(
                    ((LoggerContext) LoggerFactory.getILoggerFactory()).getProperty("logDirectory"), handle.getId()
            );
            if (sourcePath.toFile().exists() && targetDir.mkdirs()) {
                copyWithAttributes(sourcePath, targetDir.toPath());
            }
        }
    }

    /**
     * Removes a given directory if it exists.
     *
     * @param dir directory that is to be removed
     */
    private void removeDirectory(File dir) {
        if (!dir.exists()) {
            return;
        }
        File[] files = dir.listFiles();
        if (files == null) {
            return;
        }
        for (File file : files) {
            if (file.isDirectory()) {
                removeDirectory(file);
            } else if (!file.delete()) {
                this.log.error("Error while deleting file: {}", file.toString());
            }
        }
        if (!dir.delete()) {
            dir.deleteOnExit();
        }
    }

    @Override
    public String getFederationId() {
        return federation.getFederationId();
    }

    @Override
    public void setWatchdog(WatchDog watchdog) {
        watchDog = watchdog;
    }
}
